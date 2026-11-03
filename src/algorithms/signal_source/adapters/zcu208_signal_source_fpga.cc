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
#include <iterator>         // for std::begin, std::end
#include <linux/i2c-dev.h>  // I2C_FUNCS, I2C_RDWR, i2c_rdwr_ioctl_data
#include <linux/i2c.h>      // i2c_msg, I2C_FUNC_I2C
#include <numeric>          // for std::gcd
#include <stdexcept>        // for std::runtime_error
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
                throw std::runtime_error("Use an explicit /dev/i2c-N adapter path");
            }
        const auto sysdev = "/sys/bus/i2c/devices/" + node.substr(4) + "-002f/driver";
        if (access(sysdev.c_str(), F_OK) == 0)
            {
                throw std::runtime_error("Bridge 0x2f has a bound kernel driver; use its SPI interface instead.");
            }
        fd_ = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
            {
                throw std::runtime_error("Cannot open " + path + ": " + std::strerror(errno));
            }
        if (flock(fd_, LOCK_EX | LOCK_NB) < 0)
            {
                const std::string error = std::strerror(errno);
                close(fd_);
                fd_ = -1;
                throw std::runtime_error("Failed to lock I2C adapter: " + error);
            }
        unsigned long funcs = 0;
        if (ioctl(fd_, I2C_FUNCS, &funcs) < 0 || !(funcs & I2C_FUNC_I2C))
            {
                close(fd_);
                fd_ = -1;
                throw std::runtime_error("Adapter does not support I2C_RDWR transfers");
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
                throw std::runtime_error(std::string("I2C transfer to CLK104 bridge 0x2f failed: ") + std::strerror(errno));
            }
        if (r != 1)
            {
                throw std::runtime_error("Incomplete I2C transfer");
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

    uint64_t sampling_frequency_hz = configuration->property(role + ".sampling_frequency_hz", DEFAULT_SAMPLING_FREQUENCY_HZ);
    uint32_t reference_clock_frequency_hz = configuration->property(role + ".reference_clock_frequency_hz", DEFAULT_REFERENCE_CLOCK_FREQUENCY_HZ);
    uint32_t reference_clock_input = configuration->property(role + ".reference_clock_input", DEFAULT_REFERENCE_CLOCK_INPUT);

    try
        {
            // Frequencies are in Hz; RFDC configuration is performed separately.
            const auto plan = compute_clock_plan(reference_clock_frequency_hz, sampling_frequency_hz, reference_clock_input);
            print_plan(plan);
            Bridge bridge(bus);
            bridge.setup();
            std::cout << "Programming LMK (SS1)...\n";
            for (auto w : plan.lmk_registers) bridge.reg(0x02, w);
            delay(2000);  // Settle PLL1/PLL2. A delay does NOT prove lock.
            std::cout << "Programming ADC LMX (SS3)...\n";
            for (unsigned i = 0; i < plan.lmx_registers.size(); ++i)
                {
                    bridge.reg(0x08, plan.lmx_registers[i]);
                    // Extra time after reset and between the two final FCAL writes.
                    if (i == 0 || i == 1 || plan.lmx_registers[i] >> 16 == 0) delay(10);
                }
            delay(200);
            std::cout << "Register writes completed. PLL lock and frequency are NOT automatically verified.\n"
                         "Check DS1 (LMK) and DS2 (ADC LMX), and measure output before ADC acquisition.\n";
        }
    catch (const std::exception& e)
        {
            LOG(ERROR) << "Problem configuring CLK104: " << e.what();
            throw;
        }

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

    // Power down CLK104 clock generation after stopping the FPGA buffer monitor and bit-selection threads.
    if (rf_shutdown_)
        {
            try
                {
                    Bridge bridge(bus);
                    bridge.setup();
                    const auto power_down = [&bridge](unsigned char select, uint32_t word, const char* device) {
                        try
                            {
                                bridge.reg(select, word);
                            }
                        catch (const std::exception& e)
                            {
                                LOG(WARNING) << "Problem shutting down CLK104 " << device << ": " << e.what();
                            }
                    };
                    // LMX2594 R0: set POWERDOWN and clear FCAL_EN, preserving the other bits.
                    const auto lmx_r0 = lmx_profile[sizeof(lmx_profile) / sizeof(lmx_profile[0]) - 1];
                    power_down(0x08, (lmx_r0 & ~uint32_t{0x08}) | uint32_t{0x01}, "ADC LMX");
                    // LMK register 0x0002: POWERDOWN=1. Disable the reference last.
                    power_down(0x02, 0x000201u, "LMK");
                }
            catch (const std::exception& e)
                {
                    LOG(WARNING) << "Problem accessing CLK104 for shutdown: " << e.what();
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

ZCU208SignalSourceFPGA::ClockPlan ZCU208SignalSourceFPGA::compute_clock_plan(
    uint64_t reference_frequency_hz, uint64_t sampling_frequency_hz, uint32_t reference_input)
{
    constexpr uint64_t vcxo_hz = 160000000;
    constexpr uint64_t lmx_reference_hz = 160000000;
    constexpr uint64_t max_divider = 0x3FFF;
    if (reference_frequency_hz < 1000000 || reference_frequency_hz > 500000000 || reference_input > 2)
        {
            throw std::invalid_argument("CLK104 requires a 1-500 MHz reference on CLKin0, CLKin1 or CLKin2");
        }
    if (sampling_frequency_hz < 10000000 || sampling_frequency_hz > 7500000000)
        {
            throw std::invalid_argument("CLK104 planner supports ADC clocks from 10 MHz to 7.5 GHz through the LMX channel divider");
        }

    ClockPlan plan{};
    plan.reference_frequency_hz = reference_frequency_hz;
    plan.sampling_frequency_hz = sampling_frequency_hz;
    plan.reference_input = reference_input;
    // Reduce reference/R = VCXO/N, then scale both dividers to keep PLL1's
    // PFD at or below the original 80 kHz. Use integer-Hz arithmetic throughout.
    const auto common_hz = std::gcd(reference_frequency_hz, vcxo_hz);
    const auto base_r = reference_frequency_hz / common_hz;
    const auto base_n = vcxo_hz / common_hz;
    const auto max_scale = std::min(max_divider / base_r, max_divider / base_n);
    for (uint64_t scale = (common_hz + 79999) / 80000; scale <= max_scale; ++scale)
        {
            if (common_hz % scale == 0 && common_hz / scale >= 10000)
                {
                    plan.pll1_r = base_r * scale;
                    plan.pll1_n = base_n * scale;
                    plan.pll1_pfd_hz = common_hz / scale;
                    break;
                }
        }
    if (plan.pll1_r == 0)
        {
            throw std::invalid_argument("Reference cannot lock the 160 MHz VCXO with 14-bit dividers and a 10-80 kHz PLL1 PFD");
        }

    // TI LMX2594 tables 2 and 8: legal channel dividers, VCO limits and
    // integer-N delay selection. Prefer the lowest divider giving an exact N.
    constexpr uint32_t channel_dividers[] = {2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 72, 96, 128, 192, 256, 384, 512, 768};
    uint32_t channel_code = 0;
    for (const auto divider : channel_dividers)
        {
            const auto vco_hz = sampling_frequency_hz * divider;
            const uint64_t max_vco_hz = divider <= 6 ? 15000000000ULL : 11500000000ULL;
            if (vco_hz >= 7500000000ULL && vco_hz <= max_vco_hz && vco_hz % lmx_reference_hz == 0)
                {
                    plan.lmx_n = vco_hz / lmx_reference_hz;
                    plan.lmx_channel_divider = divider;
                    plan.lmx_vco_hz = vco_hz;
                    break;
                }
            ++channel_code;
        }
    if (plan.lmx_channel_divider == 0)
        {
            throw std::invalid_argument("Sampling frequency has no exact integer-N LMX plan with a 160 MHz reference");
        }
    const uint32_t pfd_delay = plan.lmx_vco_hz <= 12500000000ULL ? 1 : 2;
    const uint32_t min_n = pfd_delay == 1 ? 28 : 32;
    if (plan.lmx_n < min_n || plan.lmx_n > 0x7FFFF)
        {
            throw std::invalid_argument("LMX feedback divider is outside the supported integer-N range");
        }

    plan.lmk_registers.assign(std::begin(lmk_profile), std::end(lmk_profile));
    plan.lmx_registers.assign(std::begin(lmx_profile), std::end(lmx_profile));
    // Change only frequency/input fields; retain fixed bits and write order.
    const auto patch = [](std::vector<uint32_t>& registers, uint32_t address,
                           unsigned shift, uint32_t mask, uint32_t value) {
        bool found = false;
        for (auto& word : registers)
            {
                if (word >> shift == address)
                    {
                        word = (word & ~mask) | (value & mask);
                        found = true;
                    }
            }
        if (!found) throw std::logic_error("CLK104 profile is missing a required register");
    };
    const auto r_address = 0x153u + 2 * reference_input;
    patch(plan.lmk_registers, r_address, 8, 0x3F, plan.pll1_r >> 8);
    patch(plan.lmk_registers, r_address + 1, 8, 0xFF, plan.pll1_r);
    patch(plan.lmk_registers, 0x159, 8, 0x3F, plan.pll1_n >> 8);
    patch(plan.lmk_registers, 0x15A, 8, 0xFF, plan.pll1_n);
    // Enable the selected input, preserving the other input enables/types.
    patch(plan.lmk_registers, 0x146, 8, 1u << (3 + reference_input), 1u << (3 + reference_input));
    patch(plan.lmk_registers, 0x147, 8, 0x70, reference_input << 4);
    if (reference_input < 2)
        {
            const auto mux_shift = reference_input * 2;
            patch(plan.lmk_registers, 0x147, 8, 3u << mux_shift, 2u << mux_shift);
        }
    patch(plan.lmx_registers, 34, 16, 0x07, plan.lmx_n >> 16);
    patch(plan.lmx_registers, 36, 16, 0xFFFF, plan.lmx_n);
    patch(plan.lmx_registers, 37, 16, 0x3F00, pfd_delay << 8);
    patch(plan.lmx_registers, 31, 16, 0x4000, plan.lmx_channel_divider > 2 ? 0x4000 : 0);
    patch(plan.lmx_registers, 75, 16, 0x07C0, channel_code << 6);
    return plan;
}

void ZCU208SignalSourceFPGA::print_plan(const ClockPlan& plan)
{
    std::cout << "Reference: " << plan.reference_frequency_hz << " Hz (CLKin" << plan.reference_input << ")\n"
              << "LMK PLL1: R=" << plan.pll1_r << ", N=" << plan.pll1_n
              << ", PFD=" << plan.pll1_pfd_hz << " Hz; VCXO=160 MHz\n"
              << "LMK PLL2: R=2, P=2, N=N_CAL=16; VCO0=2560 MHz\n"
              << "LMK DCLKout0: /16 =160 MHz -> ADC LMX2594\n"
              << "LMX: R_PRE=1, R=1, doubler off, MULT=1; PFD=160 MHz\n"
              << "LMX: integer N=" << plan.lmx_n << ", VCO=" << plan.lmx_vco_hz
              << " Hz, CHDIV=" << plan.lmx_channel_divider << " -> " << plan.sampling_frequency_hz << " Hz\n"
              << "ADC LMX output A: enabled, power code 10; output B: disabled\n"
              << "Connect CLK104 J27/J28 (output A) to ZCU208 J8/J98, matching P/N.\n"
              << "Configure RFDC separately for " << plan.sampling_frequency_hz << " samples/s, PLL bypassed.\n"
              << "PLL1 loop-filter response and lock time must be validated for the selected reference.\n"
              << "This profile changes shared LMK clocks, including PL/DAC references.\n";
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
