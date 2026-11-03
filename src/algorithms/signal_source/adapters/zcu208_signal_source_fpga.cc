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
#include <algorithm>        // for std::max
#include <cerrno>           // errno, EINTR
#include <chrono>           // for std::chrono
#include <cmath>            // for std::floor
#include <cstring>          // std::strerror
#include <exception>        // for std::exception
#include <fcntl.h>          // open, O_RDWR, O_CLOEXEC
#include <iostream>         // for std::cout
#include <linux/i2c-dev.h>  // I2C_FUNCS, I2C_RDWR, i2c_rdwr_ioctl_data
#include <linux/i2c.h>      // i2c_msg, I2C_FUNC_I2C
#include <sys/file.h>       // For flock(), LOCK_EX, LOCK_NB, and LOCK_UN
#include <sys/ioctl.h>      // ioctl
#include <unistd.h>         // access, close

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/check.h>
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

void delay(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

class Bridge
{
    int fd_ = -1;

public:
    explicit Bridge(const std::string& path)
    {
        // Require a Linux child adapter of the board's I2C mux, already routed
        // by the kernel. Never manually race the kernel for the mux register.
        auto slash = path.find_last_of('/');
        auto node = path.substr(slash == std::string::npos ? 0 : slash + 1);
        if (node.rfind("i2c-", 0) != 0 || node.size() == 4 ||
            node.substr(4).find_first_not_of("0123456789") != std::string::npos)
            {
                std::cout << "Use an explicit /dev/i2c-N adapter path" << std::endl;
                exit(0);
            }
        const auto sysdev = "/sys/bus/i2c/devices/" + node.substr(4) + "-002f/driver";
        if (access(sysdev.c_str(), F_OK) == 0)
            {
                std::cout << "Bridge 0x2f has a bound kernel driver; use its SPI interface instead." << std::endl;
                exit(0);
            }
        fd_ = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
            {
                std::cout << "open " << path << std::endl;
                exit(0);
            }
        if (flock(fd_, LOCK_EX | LOCK_NB) < 0)
            {
                std::cout << "Failed to lock I2C adapter: " << std::strerror(errno) << std::endl;
                close(fd_);
                fd_ = -1;
                exit(0);
            }
        unsigned long funcs = 0;
        if (ioctl(fd_, I2C_FUNCS, &funcs) < 0 || !(funcs & I2C_FUNC_I2C))
            {
                close(fd_);
                fd_ = -1;
                std::cout << "Adapter does not support I2C_RDWR transfers" << std::endl;
                exit(0);
            }
    }
    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;
    ~Bridge()
    {
        if (fd_ >= 0) close(fd_);
    }
    void send(unsigned char* data, unsigned short count)
    {
        i2c_msg m{};
        m.addr = 0x2f;
        m.len = count;
        m.buf = data;
        i2c_rdwr_ioctl_data x{&m, 1};
        int r;
        do
            {
                r = ioctl(fd_, I2C_RDWR, &x);
            }
        while (r < 0 && errno == EINTR);
        if (r < 0)
            {
                std::cout << "I2C transfer to CLK104 bridge 0x2f" << std::endl;
                exit(0);
            }
        if (r != 1)
            {
                std::cout << "Incomplete I2C transfer" << std::endl;
                exit(0);
            }
        delay(2);  // Bridge SPI transaction must complete before another command.
    }
    void setup()
    {
        unsigned char b[] = {0xf0, 0x03};
        send(b, 2);
    }
    void reg(unsigned char select, uint32_t word)
    {
        unsigned char b[] = {select, static_cast<unsigned char>(word >> 16),
            static_cast<unsigned char>(word >> 8), static_cast<unsigned char>(word)};
        send(b, 4);
    }
};

ZCU208SignalSourceFPGA::ZCU208SignalSourceFPGA(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "ZCU208_Signal_Source_FPGA"s),
      bus(configuration->property(role + ".i2c", std::string{})),
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
    // basic checks

    CHECK(rx1_enable_ || rx2_enable_) << "At least one RX channel must be enabled.";

    const uint32_t num_freq_bands = (rx1_enable_ && rx2_enable_) ? 2 : 1;

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }

    // Resolve the CLK104 mux channel's sysfs link to its /dev/i2c-N device path.
    if (bus.empty())
        {
            bus = find_clk104_i2c();
        }
    std::cout << "using device " << bus << std::endl;

    // configure CLK104 board

    print_plan();

    Bridge bridge(bus);
    bridge.setup();
    std::cout << "Programming LMK (SS1)...\n";
    for (auto w : lmk_profile) bridge.reg(0x02, w);
    delay(2000);  // Settle PLL1/PLL2. A delay does NOT prove lock.
    std::cout << "Programming ADC LMX (SS3)...\n";
    for (unsigned i = 0; i < sizeof(lmx_profile) / sizeof(lmx_profile[0]); ++i)
        {
            bridge.reg(0x08, lmx_profile[i]);
            // Extra time after reset and between the two final FCAL writes.
            if (i == 0 || i == 1 || lmx_profile[i] >> 16 == 0) delay(10);
        }
    delay(200);
    std::cout << "Register writes completed. PLL lock and frequency are NOT automatically verified.\n"
                 "Check DS1 (LMK) and DS2 (ADC LMX), and measure output before ADC acquisition.\n";

    // configure FPGA switch, buffer monitor and dynamic bits selector

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

std::string ZCU208SignalSourceFPGA::find_clk104_i2c()
{
    // Resolve the CLK104 mux channel's sysfs link to its /dev/i2c-N device path.
    std::error_code ec;
    const auto adapter = std::filesystem::canonical(LINK, ec);

    if (ec)
        {
            std::cout << "Cannot resolve CLK104 I2C link: "
                      << LINK.string() << ": " << ec.message()
                      << std::endl;
            std::exit(0);
        }

    const auto device =
        std::filesystem::path("/dev") / adapter.filename();

    const bool exists = std::filesystem::exists(device, ec);

    if (ec)
        {
            std::cout << "Cannot check CLK104 I2C device: "
                      << device.string() << ": " << ec.message()
                      << std::endl;
            std::exit(0);
        }

    if (!exists)
        {
            std::cout << "CLK104 I2C device not found: "
                      << device.string() << std::endl;
            std::exit(0);
        }

    return device.string();
}

void ZCU208SignalSourceFPGA::print_plan()
{
    std::cout << "Reference: CLK104 onboard U4 TCXO, 10.000 MHz (CLKin1)\n"
                 "LMK PLL1: R=125, N=2000, PFD=80 kHz; VCXO=160 MHz\n"
                 "LMK PLL2: R=2, P=2, N=N_CAL=16; VCO0=2560 MHz\n"
                 "LMK DCLKout0: /16 =160 MHz -> ADC LMX2594\n"
                 "LMX: R_PRE=1, R=1, doubler off, MULT=1; PFD=160 MHz\n"
                 "LMX: integer N=56, VCO=8960 MHz, CHDIV=2 ->4480 MHz\n"
                 "ADC LMX output A: enabled, power code 10; output B: disabled\n"
                 "Connect CLK104 J27/J28 (output A) to ZCU208 J8/J98, matching P/N.\n"
                 "RFDC tile1/bank225 must separately be set to 4.480 GS/s, PLL bypassed.\n"
                 "This profile changes shared LMK clocks, including PL/DAC references.\n";
}


void ZCU208SignalSourceFPGA::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "ZCU208 FPGA source nothing to connect";
}


void ZCU208SignalSourceFPGA::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "ZCU208 FPGA source nothing to disconnect";
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
