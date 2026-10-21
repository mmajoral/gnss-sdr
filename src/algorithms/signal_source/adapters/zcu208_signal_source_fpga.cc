/*!
 * \file zcu208_signal_source_fpga.cc
 * \brief Signal source for the Analog Devices ZCU208 evaluation board
 * directly connected to the FPGA accelerators.
 * This source implements only the ZCU208 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 * \authors <ul>
 *          <li> Marc Majoral, mmajoral(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "zcu208_signal_source_fpga.h"
#include "GPS_L1_CA.h"
#include "GPS_L5.h"
#include "ad9361_manager.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include <algorithm>  // for std::max
#include <chrono>     // for std::chrono
#include <cmath>      // for std::floor
#include <exception>  // for std::exception
#include <iostream>   // for std::cout

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/check.h>
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

ZCU208SignalSourceFPGA::ZCU208SignalSourceFPGA(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "ADRV9361_Z7035_Signal_Source_FPGA"s),
      in_stream_(in_stream),
      out_stream_(out_stream),
      item_size_(sizeof(int8_t)),
      rx1_enable_(configuration->property(role + ".rx1_enable", true)),
      rx2_enable_(configuration->property(role + ".rx2_enable", true)),
      enable_dynamic_bit_selection_(configuration->property(role + ".enable_dynamic_bit_selection", true)),
      enable_ovf_check_buffer_monitor_active_(true),
      dump_(configuration->property(role + ".dump", false)),
#if USE_GLOG_AND_GFLAGS
      rf_shutdown_(configuration->property(role + ".rf_shutdown", FLAGS_rf_shutdown))
#else
      rf_shutdown_(configuration->property(role + ".rf_shutdown", absl::GetFlag(FLAGS_rf_shutdown)))
#endif
{
    CHECK(rx1_enable_ || rx2_enable_) << "At least one RX channel must be enabled.";

    const uint32_t num_freq_bands = (rx1_enable_ && rx2_enable_) ? 2 : 1;

    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(REAL_TIME_MODE);

    std::string dump_filename = configuration->property(role + ".dump_filename", DEFAULT_DUMP_FILENAME);

    buffer_monitor_fpga = std::make_shared<Fpga_buffer_monitor>(num_freq_bands, dump_, dump_filename);
    thread_buffer_monitor = std::thread([&] { run_buffer_monitor_process(); });

    // dynamic bits selection
    if (enable_dynamic_bit_selection_)
        {
            dynamic_bit_selection_fpga = std::make_shared<Fpga_dynamic_bit_selection>(rx1_enable_, rx2_enable_);
            thread_dynamic_bit_selection = std::thread([&] { run_dynamic_bit_selection_process(); });
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}

ZCU208SignalSourceFPGA::~ZCU208SignalSourceFPGA()
{
    // cleanup and exit
    if (rf_shutdown_)
        {
            // to do: shutdown rf
        }

    // disable buffer overflow checking and buffer monitoring
    {
        std::lock_guard<std::mutex> lock_buffer_monitor(buffer_monitor_mutex);
        enable_ovf_check_buffer_monitor_active_ = false;
    }

    if (thread_buffer_monitor.joinable())
        {
            thread_buffer_monitor.join();
        }
    bool bit_selection_enabled = false;
    {
        std::lock_guard<std::mutex> lock_dyn_bit_sel(dynamic_bit_selection_mutex);
        bit_selection_enabled = enable_dynamic_bit_selection_;
    }

    if (bit_selection_enabled == true)
        {
            {
                std::lock_guard<std::mutex> lock(dynamic_bit_selection_mutex);
                enable_dynamic_bit_selection_ = false;
            }

            if (thread_dynamic_bit_selection.joinable())
                {
                    thread_dynamic_bit_selection.join();
                }
        }
}


void ZCU208SignalSourceFPGA::run_dynamic_bit_selection_process()
{
    bool dynamic_bit_selection_active = true;

    while (dynamic_bit_selection_active)
        {
            // setting the bit selection to the top bits
            dynamic_bit_selection_fpga->bit_selection();
            std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_CONTROL_PERIOD_ms));
            std::lock_guard<std::mutex> lock(dynamic_bit_selection_mutex);
            if (enable_dynamic_bit_selection_ == false)
                {
                    dynamic_bit_selection_active = false;
                }
        }
}

void ZCU208SignalSourceFPGA::run_buffer_monitor_process()
{
    bool enable_ovf_check_buffer_monitor_active = true;

    std::this_thread::sleep_for(std::chrono::milliseconds(BUFFER_MONITOR_INITIAL_DELAY_ms));

    while (enable_ovf_check_buffer_monitor_active)
        {
            if (buffer_monitor_fpga->check_buffer_overflow_and_monitor_buffer_status())
                {
                    // If a buffer overflow is detected, the receiver may not function correctly.
                    // This compromises system reliability and can lead to undefined behavior.
                    // To prevent further issues, execution is halted.
                    LOG(FATAL) << "Buffer Overflow Detected – Execution Halted";
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(BUFFER_MONITOR_PERIOD_ms));
            std::lock_guard<std::mutex> lock(buffer_monitor_mutex);
            if (enable_ovf_check_buffer_monitor_active_ == false)
                {
                    enable_ovf_check_buffer_monitor_active = false;
                }
        }
}

void ZCU208SignalSourceFPGA::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void ZCU208SignalSourceFPGA::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr ZCU208SignalSourceFPGA::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr ZCU208SignalSourceFPGA::get_right_block()
{
    return {};
}
