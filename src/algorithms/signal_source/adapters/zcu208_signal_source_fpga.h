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
#include <vector>

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

    struct ClockPlan
    {
        uint64_t reference_frequency_hz;
        uint64_t sampling_frequency_hz;
        uint32_t reference_input;
        uint32_t pll1_r;
        uint32_t pll1_n;
        uint64_t pll1_pfd_hz;
        uint32_t lmx_n;
        uint32_t lmx_channel_divider;
        uint64_t lmx_vco_hz;
        std::vector<uint32_t> lmk_registers;
        std::vector<uint32_t> lmx_registers;
    };

    // Compute exact integer-N settings without accessing hardware or configuring RFDC.
    // Assumes the existing 160 MHz VCXO, LMK PLL2 and 160 MHz LMX reference.
    // reference_input selects CLKin0/1/2; the caller must route and electrically
    // configure that input for the reference. Default CLKin1 is the onboard TCXO.
    // PLL1 PFD is restricted to 10-80 kHz; loop-filter/lock performance still needs
    // board validation. Unsupported frequencies throw std::invalid_argument.
    static ClockPlan compute_clock_plan(uint64_t reference_frequency_hz,
        uint64_t sampling_frequency_hz, uint32_t reference_input = 1);

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

    // default value for the sampling frequency configured in the RFADC in the FPGA
    const uint64_t DEFAULT_SAMPLING_FREQUENCY_HZ = 4480000000;

    // Default reference clock frequency
    const uint32_t DEFAULT_REFERENCE_CLOCK_FREQUENCY_HZ = 10000000;

    // Default reference clock input:
    // 0 for CLKin0,
    // 1 for CLKin1 (onboard 10 MHz TCXO in the current default setup,
    // 2 for CLKin2
    const uint32_t DEFAULT_REFERENCE_CLOCK_INPUT = 1;

    // Sysfs symlink to the CLK104 I2C adapter on channel 5 of the
    // I2C multiplexer at address 0x74 on bus 1.
    // Resolved at runtime to determine the corresponding /dev/i2c-N device.
    const std::filesystem::path LINK = "/sys/bus/i2c/devices/1-0074/channel-5";

    // Ordered register configuration for the CLK104 LMK clock device.
    // Test reference: CLK104 onboard 10 MHz TCXO on CLKin1.
    // Register definitions: https://www.ti.com/lit/ds/symlink/lmk04828.pdf
    // Configures a 160 MHz reference clock for the ADC LMX2594.
    // Each 24-bit word contains a register address and its 8-bit value.
    // Follow the writable-register sequence in the datasheet.
    // Omit read-only IDs, addresses absent from the register map, and SPI_LOCK
    // writes (the lock defaults to unlocked). Keep the fixed/optimization writes.
    static constexpr uint32_t lmk_profile[] = {
        0x000090u,  // 0x0000 = 0x90; Assert software reset; disable 3-wire SPI.
        0x000010u,  // 0x0000 = 0x10; Normal operation; 3-wire SPI remains disabled.
        0x000200u,  // 0x0002 = 0x00; POWERDOWN=0: normal operation.
        0x010070u,  // 0x0100 = 0x70; DCLKout0_DIV=16: 2560 MHz / 16 = 160 MHz for ADC LMX.
        0x010155u,  // 0x0101 = 0x55; DCLKout0 digital delay: high count=5, low count=5.
        0x010301u,  // 0x0103 = 0x01; DCLKout0 analog delay code=0; delay mux=0; clock mux=1.
        0x010422u,  // 0x0104 = 0x22; DCLKout0 half-step; SDCLKout1 source, digital delay and half-step.
        0x010500u,  // 0x0105 = 0x00; SDCLKout1 analog delay enable=0, delay code=0.
        0x010673u,  // 0x0106 = 0x73; CLKout0/1 power controls; group powered up; SDCLKout1 powered down.
        0x010703u,  // 0x0107 = 0x03; CLKout0/1 polarity and format: device format=3, SYSREF format=0.
        0x010870u,  // 0x0108 = 0x70; DCLKout2_DIV=16; output/input drive-level controls.
        0x010955u,  // 0x0109 = 0x55; DCLKout2 digital delay: high count=5, low count=5.
        0x010B00u,  // 0x010B = 0x00; DCLKout2 analog delay code=0; delay mux=0; clock mux=0.
        0x010C22u,  // 0x010C = 0x22; DCLKout2 half-step; SDCLKout3 source, digital delay and half-step.
        0x010D00u,  // 0x010D = 0x00; SDCLKout3 analog delay enable=0, delay code=0.
        0x010EF0u,  // 0x010E = 0xF0; CLKout2/3 power controls; group powered up; SDCLKout3 powered up.
        0x010F30u,  // 0x010F = 0x30; CLKout2/3 polarity and format: device format=0, SYSREF format=3.
        0x011070u,  // 0x0110 = 0x70; DCLKout4_DIV=16; output/input drive-level controls.
        0x011155u,  // 0x0111 = 0x55; DCLKout4 digital delay: high count=5, low count=5.
        0x011301u,  // 0x0113 = 0x01; DCLKout4 analog delay code=0; delay mux=0; clock mux=1.
        0x011422u,  // 0x0114 = 0x22; DCLKout4 half-step; SDCLKout5 source, digital delay and half-step.
        0x011500u,  // 0x0115 = 0x00; SDCLKout5 analog delay enable=0, delay code=0.
        0x011673u,  // 0x0116 = 0x73; CLKout4/5 power controls; group powered up; SDCLKout5 powered down.
        0x011703u,  // 0x0117 = 0x03; CLKout4/5 polarity and format: device format=3, SYSREF format=0.
        0x011870u,  // 0x0118 = 0x70; DCLKout6_DIV=16; output/input drive-level controls.
        0x011955u,  // 0x0119 = 0x55; DCLKout6 digital delay: high count=5, low count=5.
        0x011B01u,  // 0x011B = 0x01; DCLKout6 analog delay code=0; delay mux=0; clock mux=1.
        0x011C22u,  // 0x011C = 0x22; DCLKout6 half-step; SDCLKout7 source, digital delay and half-step.
        0x011D00u,  // 0x011D = 0x00; SDCLKout7 analog delay enable=0, delay code=0.
        0x011E72u,  // 0x011E = 0x72; CLKout6/7 power controls; group powered up; SDCLKout7 powered up.
        0x011F03u,  // 0x011F = 0x03; CLKout6/7 polarity and format: device format=3, SYSREF format=0.
        0x012070u,  // 0x0120 = 0x70; DCLKout8_DIV=16; output/input drive-level controls.
        0x012155u,  // 0x0121 = 0x55; DCLKout8 digital delay: high count=5, low count=5.
        0x012301u,  // 0x0123 = 0x01; DCLKout8 analog delay code=0; delay mux=0; clock mux=1.
        0x012422u,  // 0x0124 = 0x22; DCLKout8 half-step; SDCLKout9 source, digital delay and half-step.
        0x012500u,  // 0x0125 = 0x00; SDCLKout9 analog delay enable=0, delay code=0.
        0x012670u,  // 0x0126 = 0x70; CLKout8/9 power controls; group powered up; SDCLKout9 powered up.
        0x012733u,  // 0x0127 = 0x33; CLKout8/9 polarity and format: device format=3, SYSREF format=3.
        0x012870u,  // 0x0128 = 0x70; DCLKout10_DIV=16; output/input drive-level controls.
        0x012955u,  // 0x0129 = 0x55; DCLKout10 digital delay: high count=5, low count=5.
        0x012B00u,  // 0x012B = 0x00; DCLKout10 analog delay code=0; delay mux=0; clock mux=0.
        0x012C22u,  // 0x012C = 0x22; DCLKout10 half-step; SDCLKout11 source, digital delay and half-step.
        0x012D00u,  // 0x012D = 0x00; SDCLKout11 analog delay enable=0, delay code=0.
        0x012EF0u,  // 0x012E = 0xF0; CLKout10/11 power controls; group powered up; SDCLKout11 powered up.
        0x012F30u,  // 0x012F = 0x30; CLKout10/11 polarity and format: device format=0, SYSREF format=3.
        0x013070u,  // 0x0130 = 0x70; DCLKout12_DIV=16; output/input drive-level controls.
        0x013155u,  // 0x0131 = 0x55; DCLKout12 digital delay: high count=5, low count=5.
        0x013301u,  // 0x0133 = 0x01; DCLKout12 analog delay code=0; delay mux=0; clock mux=1.
        0x013422u,  // 0x0134 = 0x22; DCLKout12 half-step; SDCLKout13 source, digital delay and half-step.
        0x013500u,  // 0x0135 = 0x00; SDCLKout13 analog delay enable=0, delay code=0.
        0x013673u,  // 0x0136 = 0x73; CLKout12/13 power controls; group powered up; SDCLKout13 powered down.
        0x013703u,  // 0x0137 = 0x03; CLKout12/13 polarity and format: device format=3, SYSREF format=0.
        0x013800u,  // 0x0138 = 0x00; VCO_MUX=0: select VCO0.
        0x013903u,  // 0x0139 = 0x03; SYSREF_MUX=3: select SYSREF pulser.
        0x013A01u,  // 0x013A = 0x01; SYSREF_DIV[12:8]=1.
        0x013B2Cu,  // 0x013B = 0x2C; SYSREF_DIV[7:0]=44: total divider=300.
        0x013C00u,  // 0x013C = 0x00; SYSREF_DDLY[12:8]=0.
        0x013D01u,  // 0x013D = 0x01; SYSREF_DDLY[7:0]=1: digital delay count=1.
        0x013E03u,  // 0x013E = 0x03; SYSREF_PULSE_CNT=3: request eight pulses per burst.
        0x013F02u,  // 0x013F = 0x02; Feedback mux control: FB_MUX=1; FB_MUX_EN=0; PLL N clocks use normal paths.
        0x014009u,  // 0x0140 = 0x09; Power control: PLL1, VCO and OSCin enabled; global SYSREF and pulser powered down.
        0x014100u,  // 0x0141 = 0x00; Dynamic digital delay disabled for all outputs.
        0x014200u,  // 0x0142 = 0x00; DDLYd_STEP_CNT=0: dynamic delay step count.
        0x014331u,  // 0x0143 = 0x31; SYNC_EN=1; SYNC_MODE=1; SYSREF digital delay clear deasserted.
        0x0144FFu,  // 0x0144 = 0xFF; SYNC disabled for all clock output groups and SYSREF.
        0x01457Fu,  // 0x0145 = 0x7F; Fixed register: required value 0x7F.
        0x014618u,  // 0x0146 = 0x18; CLKin0/1 enabled; CLKin2 disabled; input electrical types configured.
        0x01471Au,  // 0x0147 = 0x1A; CLKin_SEL_MODE=1: manually select CLKin1; route CLKin1 to PLL1.
        0x014806u,  // 0x0148 = 0x06; CLKin_SEL0 pin mux and electrical type.
        0x014946u,  // 0x0149 = 0x46; CLKin_SEL1 pin mux, electrical type and SDIO readback type.
        0x014A06u,  // 0x014A = 0x06; RESET pin mux and electrical type.
        0x014B06u,  // 0x014B = 0x06; Holdover control: loss-of-signal timeout, tracking and manual DAC enable.
        0x014C00u,  // 0x014C = 0x00; MAN_DAC[7:0]=0: manual holdover DAC lower byte.
        0x014D00u,  // 0x014D = 0x00; DAC_TRIP_LOW=0: lower holdover DAC threshold.
        0x014EC0u,  // 0x014E = 0xC0; DAC_CLK_MULT=3; DAC_TRIP_HIGH=0: DAC timing and upper threshold.
        0x014F7Fu,  // 0x014F = 0x7F; DAC_CLK_CNTR=127: holdover DAC clock counter.
        0x015013u,  // 0x0150 = 0x13; Holdover detection, hitless switching and enable controls.
        0x015102u,  // 0x0151 = 0x02; HOLDOVER_DLD_CNT[13:8]=2.
        0x015200u,  // 0x0152 = 0x00; HOLDOVER_DLD_CNT[7:0]=0: holdover lock count=512.
        0x015300u,  // 0x0153 = 0x00; CLKin0_R[13:8]=0.
        0x01547Du,  // 0x0154 = 0x7D; CLKin0_R[7:0]=125: input divider=125.
        0x015500u,  // 0x0155 = 0x00; CLKin1_R[13:8]=0.
        0x01567Du,  // 0x0156 = 0x7D; CLKin1_R[7:0]=125: 10 MHz / 125 = 80 kHz.
        0x015703u,  // 0x0157 = 0x03; CLKin2_R[13:8]=3.
        0x0158C0u,  // 0x0158 = 0xC0; CLKin2_R[7:0]=192: input divider=960.
        0x015907u,  // 0x0159 = 0x07; PLL1_N[13:8]=7.
        0x015AD0u,  // 0x015A = 0xD0; PLL1_N[7:0]=208: N=2000; 80 kHz * 2000 = 160 MHz.
        0x015BDAu,  // 0x015B = 0xDA; PLL1 lock window, charge-pump polarity, tristate and gain.
        0x015C20u,  // 0x015C = 0x20; PLL1_DLD_CNT[13:8]=32.
        0x015D00u,  // 0x015D = 0x00; PLL1_DLD_CNT[7:0]=0: digital lock count=8192.
        0x015E00u,  // 0x015E = 0x00; PLL1_R_DLY=0; PLL1_N_DLY=0: no reference/feedback delay.
        0x015F3Eu,  // 0x015F = 0x3E; PLL1 status pin: mux code 7 (SPI readback), open-drain output.
        0x016000u,  // 0x0160 = 0x00; PLL2_R[11:8]=0.
        0x016102u,  // 0x0161 = 0x02; PLL2_R[7:0]=2.
        0x016248u,  // 0x0162 = 0x48; PLL2_P=2; OSCin_FREQ=2; reference doubler disabled.
        0x016300u,  // 0x0163 = 0x00; PLL2_N_CAL[17:16]=0.
        0x016400u,  // 0x0164 = 0x00; PLL2_N_CAL[15:8]=0.
        0x016510u,  // 0x0165 = 0x10; PLL2_N_CAL[7:0]=16.
        0x0171AAu,  // 0x0171 = 0xAA; Fixed register: required value 0xAA.
        0x017202u,  // 0x0172 = 0x02; Fixed register: required value 0x02.
        0x017C15u,  // 0x017C = 0x15; OPT_REG_1=21: VCO optimization setting.
        0x017D33u,  // 0x017D = 0x33; OPT_REG_2=51: VCO optimization setting.
        0x016600u,  // 0x0166 = 0x00; PLL2_N[17:16]=0; frequency calibration enabled.
        0x016700u,  // 0x0167 = 0x00; PLL2_N[15:8]=0.
        0x016810u,  // 0x0168 = 0x10; PLL2_N[7:0]=16; starts VCO calibration. 160 MHz / 2 * 2 * 16 = 2560 MHz.
        0x016959u,  // 0x0169 = 0x59; PLL2 lock window, charge-pump gain, polarity and tristate.
        0x016A20u,  // 0x016A = 0x20; SYSREF_REQ_EN=0; PLL2_DLD_CNT[13:8]=32.
        0x016B00u,  // 0x016B = 0x00; PLL2_DLD_CNT[7:0]=0: digital lock count=8192.
        0x016C00u,  // 0x016C = 0x00; PLL2_LF_R4=0; PLL2_LF_R3=0: internal loop-filter resistor settings.
        0x016D00u,  // 0x016D = 0x00; PLL2_LF_C4=0; PLL2_LF_C3=0: internal loop-filter capacitor settings.
        0x016E1Bu,  // 0x016E = 0x1B; PLL2 status pin: combined PLL1/PLL2 lock indication, push-pull output.
        0x017300u,  // 0x0173 = 0x00; PLL2 and PLL2 prescaler enabled.
    };

    // Ordered register configuration for the CLK104 ADC LMX2594 synthesizer.
    // Register definitions: https://www.ti.com/lit/ds/symlink/lmx2594.pdf
    // Configures output A to 4480 MHz using the 160 MHz LMK reference.
    // Each 24-bit word contains a register address and its 16-bit value.
    // Fixed registers have no named programmable fields in the datasheet.
    // Sequence: reset, R78 down to R0, then repeat R0 after the calibration delay.
    static constexpr uint32_t lmx_profile[] = {
        0x000002u,  // R0 = 0x0002; Assert software reset.
        0x000000u,  // R0 = 0x0000; Release software reset.
        0x4E016Fu,  // R78 = 0x016F; VCO_CAPCTRL_STRT=183; QUICK_RECAL_EN=0; ramp threshold upper bit=0.
        0x4D0000u,  // R77 = 0x0000; Required initialization value; see the datasheet register map.
        0x4C000Cu,  // R76 = 0x000C; Required initialization value; see the datasheet register map.
        0x4B0800u,  // R75 = 0x0800; CHDIV=0: divide VCO by 2; output = 4480 MHz.
        0x4A0000u,  // R74 = 0x0000; SYSREF_PULSE_CNT=0; JESD_DAC3_CTRL=0; JESD_DAC4_CTRL=0.
        0x49003Fu,  // R73 = 0x003F; JESD_DAC1_CTRL=63; JESD_DAC2_CTRL=0: SYSREF delay adjustment.
        0x480001u,  // R72 = 0x0001; SYSREF_DIV=1: SYSREF divider code.
        0x470081u,  // R71 = 0x0081; SYSREF prescaler code=4; SYSREF output and pulser disabled.
        0x46C350u,  // R70 = 0xC350; MASH_RST_COUNT[15:0]=50000.
        0x450000u,  // R69 = 0x0000; MASH_RST_COUNT[31:16]=0.
        0x4403E8u,  // R68 = 0x03E8; Required initialization value; see the datasheet register map.
        0x430000u,  // R67 = 0x0000; Required initialization value; see the datasheet register map.
        0x4201F4u,  // R66 = 0x01F4; Required initialization value; see the datasheet register map.
        0x410000u,  // R65 = 0x0000; Required initialization value; see the datasheet register map.
        0x401388u,  // R64 = 0x1388; Required initialization value; see the datasheet register map.
        0x3F0000u,  // R63 = 0x0000; Required initialization value; see the datasheet register map.
        0x3E0322u,  // R62 = 0x0322; Required initialization value; see the datasheet register map.
        0x3D00A8u,  // R61 = 0x00A8; Required initialization value; see the datasheet register map.
        0x3C0000u,  // R60 = 0x0000; LD_DLY=0: no additional lock-detect delay.
        0x3B0001u,  // R59 = 0x0001; LD_TYPE=1: lock indication includes VCO calibration and tuning status.
        0x3A8001u,  // R58 = 0x8001; SYNC/SYSREF input format and level; INPIN_IGNORE=1.
        0x390020u,  // R57 = 0x0020; Required initialization value; see the datasheet register map.
        0x380000u,  // R56 = 0x0000; Required initialization value; see the datasheet register map.
        0x370000u,  // R55 = 0x0000; Required initialization value; see the datasheet register map.
        0x360000u,  // R54 = 0x0000; Required initialization value; see the datasheet register map.
        0x350000u,  // R53 = 0x0000; Required initialization value; see the datasheet register map.
        0x340820u,  // R52 = 0x0820; Required initialization value; see the datasheet register map.
        0x330080u,  // R51 = 0x0080; Required initialization value; see the datasheet register map.
        0x320000u,  // R50 = 0x0000; Required initialization value; see the datasheet register map.
        0x314180u,  // R49 = 0x4180; Required initialization value; see the datasheet register map.
        0x300300u,  // R48 = 0x0300; Required initialization value; see the datasheet register map.
        0x2F0300u,  // R47 = 0x0300; Required initialization value; see the datasheet register map.
        0x2E07FCu,  // R46 = 0x07FC; OUTB_MUX=0: output B selects channel divider (output B is powered down).
        0x2DC0DFu,  // R45 = 0xC0DF; OUTA_MUX=0: output A uses channel divider.
        0x2C0A80u,  // R44 = 0x0A80; OUTA_PWR=10; output A enabled; output B disabled; MASH_ORDER=0.
        0x2B0000u,  // R43 = 0x0000; PLL_NUM[15:0]=0: integer frequency ratio.
        0x2A0000u,  // R42 = 0x0000; PLL_NUM[31:16]=0.
        0x290000u,  // R41 = 0x0000; MASH_SEED[15:0]=0.
        0x280000u,  // R40 = 0x0000; MASH_SEED[31:16]=0.
        0x270001u,  // R39 = 0x0001; PLL_DEN[15:0]=1.
        0x260000u,  // R38 = 0x0000; PLL_DEN[31:16]=0.
        0x250104u,  // R37 = 0x0104; PFD_DLY_SEL=1; MASH_SEED_EN=0.
        0x240038u,  // R36 = 0x0038; PLL_N[15:0]=56: VCO = 160 MHz * 56 = 8960 MHz.
        0x230004u,  // R35 = 0x0004; Required initialization value; see the datasheet register map.
        0x220000u,  // R34 = 0x0000; PLL_N[18:16]=0.
        0x211E21u,  // R33 = 0x1E21; Required initialization value; see the datasheet register map.
        0x200393u,  // R32 = 0x0393; Required initialization value; see the datasheet register map.
        0x1F03ECu,  // R31 = 0x03EC; SEG1_EN=0: channel-divider buffer setting for divide-by-2.
        0x1E318Cu,  // R30 = 0x318C; Required initialization value; see the datasheet register map.
        0x1D318Cu,  // R29 = 0x318C; Required initialization value; see the datasheet register map.
        0x1C0488u,  // R28 = 0x0488; Required initialization value; see the datasheet register map.
        0x1B0002u,  // R27 = 0x0002; Required initialization value; see the datasheet register map.
        0x1A0DB0u,  // R26 = 0x0DB0; Required initialization value; see the datasheet register map.
        0x190624u,  // R25 = 0x0624; Required initialization value; see the datasheet register map.
        0x18071Au,  // R24 = 0x071A; Required initialization value; see the datasheet register map.
        0x17007Cu,  // R23 = 0x007C; Required initialization value; see the datasheet register map.
        0x160001u,  // R22 = 0x0001; Required initialization value; see the datasheet register map.
        0x150401u,  // R21 = 0x0401; Required initialization value; see the datasheet register map.
        0x14F848u,  // R20 = 0xF848; VCO_SEL=7; VCO_SEL_FORCE=0: automatic VCO selection.
        0x1327B7u,  // R19 = 0x27B7; VCO_CAPCTRL=183: capacitor code used if forced.
        0x120064u,  // R18 = 0x0064; Required initialization value; see the datasheet register map.
        0x11012Cu,  // R17 = 0x012C; VCO_DACISET_STRT=300: amplitude calibration starting code.
        0x100080u,  // R16 = 0x0080; VCO_DACISET=128: amplitude code used if forced.
        0x0F064Fu,  // R15 = 0x064F; Required initialization value; see the datasheet register map.
        0x0E1E40u,  // R14 = 0x1E40; CPG=7: charge-pump current code.
        0x0D4000u,  // R13 = 0x4000; Required initialization value; see the datasheet register map.
        0x0C5001u,  // R12 = 0x5001; PLL_R_PRE=1: PFD frequency = 160 MHz.
        0x0B0018u,  // R11 = 0x0018; PLL_R=1.
        0x0A10D8u,  // R10 = 0x10D8; MULT=1: multiplier bypassed.
        0x090604u,  // R9 = 0x0604; OSC_2X=0: reference doubler disabled.
        0x082000u,  // R8 = 0x2000; VCO_DACISET_FORCE=0; VCO_CAPCTRL_FORCE=0: automatic VCO calibration.
        0x0740B2u,  // R7 = 0x40B2; OUT_FORCE=1: force output buffers on.
        0x06C802u,  // R6 = 0xC802; Required initialization value; see the datasheet register map.
        0x0500C8u,  // R5 = 0x00C8; Required initialization value; see the datasheet register map.
        0x040C43u,  // R4 = 0x0C43; ACAL_CMP_DLY=12: VCO amplitude calibration delay.
        0x030642u,  // R3 = 0x0642; Required initialization value; see the datasheet register map.
        0x020500u,  // R2 = 0x0500; Required initialization value; see the datasheet register map.
        0x010809u,  // R1 = 0x0809; CAL_CLK_DIV=1: calibration clock divider code.
        0x00251Cu,  // R0 = 0x251C; FCAL_EN=1: start VCO calibration.
        0x00251Cu,  // R0 = 0x251C; FCAL_EN=1: repeat calibration after the required delay.
    };

    void run_dynamic_bit_selection_process();
    void run_buffer_monitor_process();
    std::string find_clk104_i2c();
    void print_plan(const ClockPlan &plan);

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
