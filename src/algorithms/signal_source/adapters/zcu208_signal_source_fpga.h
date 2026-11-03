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
#include <filesystem>
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

    // FPGA switch position for real-time receiver operation.
    const int32_t REAL_TIME_MODE = 2;

    // Sysfs symlink to the CLK104 I2C adapter on channel 5 of the
    // I2C multiplexer at address 0x74 on bus 1.
    // Resolved at runtime to determine the corresponding /dev/i2c-N device.
    const std::filesystem::path LINK = "/sys/bus/i2c/devices/1-0074/channel-5";

    // Ordered register configuration for the CLK104 LMK clock device.
    // Configures a 160 MHz reference clock for the ADC LMX2594.
    // Each 24-bit word contains a register address and its 8-bit value.
    const uint32_t lmk_profile[128] = {
        0x000090u, 0x000010u, 0x000200u, 0x000306u, 0x0004D0u, 0x00055Bu,
        0x000600u, 0x000C51u, 0x000D04u, 0x010070u, 0x010155u, 0x010255u,
        0x010301u, 0x010422u, 0x010500u, 0x010673u, 0x010703u, 0x010870u,
        0x010955u, 0x010A55u, 0x010B00u, 0x010C22u, 0x010D00u, 0x010EF0u,
        0x010F30u, 0x011070u, 0x011155u, 0x011255u, 0x011301u, 0x011422u,
        0x011500u, 0x011673u, 0x011703u, 0x011870u, 0x011955u, 0x011A55u,
        0x011B01u, 0x011C22u, 0x011D00u, 0x011E72u, 0x011F03u, 0x012070u,
        0x012155u, 0x012255u, 0x012301u, 0x012422u, 0x012500u, 0x012670u,
        0x012733u, 0x012870u, 0x012955u, 0x012A55u, 0x012B00u, 0x012C22u,
        0x012D00u, 0x012EF0u, 0x012F30u, 0x013070u, 0x013155u, 0x013255u,
        0x013301u, 0x013422u, 0x013500u, 0x013673u, 0x013703u, 0x013800u,
        0x013903u, 0x013A01u, 0x013B2Cu, 0x013C00u, 0x013D01u, 0x013E03u,
        0x013F02u, 0x014009u, 0x014100u, 0x014200u, 0x014331u, 0x0144FFu,
        0x01457Fu, 0x014618u, 0x01471Au, 0x014806u, 0x014946u, 0x014A06u,
        0x014B06u, 0x014C00u, 0x014D00u, 0x014EC0u, 0x014F7Fu, 0x015013u,
        0x015102u, 0x015200u, 0x015300u, 0x01547Du, 0x015500u, 0x01567Du,
        0x015703u, 0x0158C0u, 0x015907u, 0x015AD0u, 0x015BDAu, 0x015C20u,
        0x015D00u, 0x015E00u, 0x015F3Eu, 0x016000u, 0x016102u, 0x016248u,
        0x016300u, 0x016400u, 0x016510u, 0x0171AAu, 0x017202u, 0x017C15u,
        0x017D33u, 0x016600u, 0x016700u, 0x016810u, 0x016959u, 0x016A20u,
        0x016B00u, 0x016C00u, 0x016D00u, 0x016E1Bu, 0x017300u, 0x1FFD00u,
        0x1FFE00u, 0x1FFF53u};

    // Ordered register configuration for the CLK104 ADC LMX2594 synthesizer.
    // Configures output A to 4480 MHz using the 160 MHz LMK reference.
    // Each 24-bit word contains a register address and its 16-bit value.
    const uint32_t lmx_profile[116] = {
        0x000002u, 0x000000u, 0x700000u, 0x6F0000u, 0x6E0000u, 0x6D0000u,
        0x6C0000u, 0x6B0000u, 0x6A0000u, 0x690021u, 0x680000u, 0x670000u,
        0x663F80u, 0x650011u, 0x640000u, 0x630000u, 0x620200u, 0x610888u,
        0x600000u, 0x5F0000u, 0x5E0000u, 0x5D0000u, 0x5C0000u, 0x5B0000u,
        0x5A0000u, 0x590000u, 0x580000u, 0x570000u, 0x560000u, 0x55D300u,
        0x540001u, 0x530000u, 0x521E00u, 0x510000u, 0x506666u, 0x4F0026u,
        0x4E016Fu, 0x4D0000u, 0x4C000Cu, 0x4B0800u, 0x4A0000u, 0x49003Fu,
        0x480001u, 0x470081u, 0x46C350u, 0x450000u, 0x4403E8u, 0x430000u,
        0x4201F4u, 0x410000u, 0x401388u, 0x3F0000u, 0x3E0322u, 0x3D00A8u,
        0x3C0000u, 0x3B0001u, 0x3A8001u, 0x390020u, 0x380000u, 0x370000u,
        0x360000u, 0x350000u, 0x340820u, 0x330080u, 0x320000u, 0x314180u,
        0x300300u, 0x2F0300u, 0x2E07FCu, 0x2DC0DFu, 0x2C0A80u, 0x2B0000u,
        0x2A0000u, 0x290000u, 0x280000u, 0x270001u, 0x260000u, 0x250104u,
        0x240038u, 0x230004u, 0x220000u, 0x211E21u, 0x200393u, 0x1F03ECu,
        0x1E318Cu, 0x1D318Cu, 0x1C0488u, 0x1B0002u, 0x1A0DB0u, 0x190624u,
        0x18071Au, 0x17007Cu, 0x160001u, 0x150401u, 0x14F848u, 0x1327B7u,
        0x120064u, 0x11012Cu, 0x100080u, 0x0F064Fu, 0x0E1E40u, 0x0D4000u,
        0x0C5001u, 0x0B0018u, 0x0A10D8u, 0x090604u, 0x082000u, 0x0740B2u,
        0x06C802u, 0x0500C8u, 0x040C43u, 0x030642u, 0x020500u, 0x010809u,
        0x00251Cu, 0x00251Cu};

    void run_dynamic_bit_selection_process();
    void run_buffer_monitor_process();
    std::string find_clk104_i2c();
    void print_plan();

    mutable std::mutex dynamic_bit_selection_mutex;
    mutable std::mutex buffer_monitor_mutex;

    std::thread thread_dynamic_bit_selection;
    std::thread thread_buffer_monitor;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;
    std::shared_ptr<Fpga_buffer_monitor> buffer_monitor_fpga;

    std::string bus;

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
