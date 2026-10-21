/*!
 * \file zcu208_signal_source_fpga.h
 * \brief Signal source for the ZCU208 evaluation board connected directly
 * to FPGA accelerators.
 * This source implements only the ZCU208 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
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

#ifndef GNSS_SDR_ZCU208_SIGNAL_SOURCE_FPGA_H
#define GNSS_SDR_ZCU208_SIGNAL_SOURCE_FPGA_H

#include "concurrent_queue.h"
#include "fpga_buffer_monitor.h"
#include "fpga_dynamic_bit_selection.h"
#include "fpga_switch.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

class ConfigurationInterface;

class ZCU208SignalSourceFPGA : public SignalSourceBase
{
public:
    ZCU208SignalSourceFPGA(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~ZCU208SignalSourceFPGA();

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string DEFAULT_DUMP_FILENAME = std::string("FPGA_buffer_monitor_dump.dat");

    // perform dynamic bit selection every 500 ms by default
    const uint32_t GAIN_CONTROL_PERIOD_ms = 500;
    // check buffer overflow and perform buffer monitoring every 1s by default
    const uint32_t BUFFER_MONITOR_PERIOD_ms = 1000;
    // buffer overflow and buffer monitoring initial delay
    const uint32_t BUFFER_MONITOR_INITIAL_DELAY_ms = 2000;
    const int32_t REAL_TIME_MODE = 2;

    void run_dynamic_bit_selection_process();
    void run_buffer_monitor_process();

    mutable std::mutex dynamic_bit_selection_mutex;
    mutable std::mutex buffer_monitor_mutex;

    std::thread thread_dynamic_bit_selection;
    std::thread thread_buffer_monitor;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;
    std::shared_ptr<Fpga_buffer_monitor> buffer_monitor_fpga;

    uint32_t in_stream_;
    uint32_t out_stream_;

    size_t item_size_;

    bool rx1_enable_;
    bool rx2_enable_;

    bool enable_dynamic_bit_selection_;
    bool enable_ovf_check_buffer_monitor_active_;
    bool dump_;
    bool rf_shutdown_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_ZCU208_SIGNAL_SOURCE_FPGA_H
