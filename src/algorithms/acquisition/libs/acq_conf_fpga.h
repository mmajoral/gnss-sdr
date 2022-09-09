/*!
 * \file acq_conf_fpga.h
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm running in the FPGA.
 * \author Marc Majoral, 2022. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQ_CONF_FPGA_H
#define GNSS_SDR_ACQ_CONF_FPGA_H

#include "configuration_interface.h"
#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs acquisition_libs
 * Library with utilities for GNSS signal acquisition
 * \{ */


class Acq_Conf_Fpga
{
public:
    Acq_Conf_Fpga() = default;

    // normal configuration
    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role, uint32_t downs_factor, uint32_t sel_queue_fpga, uint32_t blk_exp, double chip_rate, double code_length_chips);

    // high sensitivity configuration
    void SetFromHSConfiguration(const ConfigurationInterface *configuration, const std::string &role, uint32_t downs_factor, uint32_t sel_queue_fpga, double chip_rate);

    /* PCPS Acquisition configuration */
    std::string device_name = "uio0";
    int64_t fs_in{4000000LL};
    float doppler_step{250.0};
    float doppler_step2{125.0};
    uint32_t samples_per_code{1U};
    uint32_t num_doppler_bins_step2{4U};
    int32_t doppler_max{5000};
    bool make_2_steps{false};

    /* PCPS Normal Acquisition configuration */
    uint32_t *all_fft_codes = NULL;  // pointer to memory that contains all the code ffts
    uint32_t select_queue_Fpga{0U};
    uint32_t downsampling_factor{4U};
    uint32_t total_block_exp{13U};
    uint32_t excludelimit{5U};
    uint32_t max_num_acqs{2U};
    uint32_t code_length{16000U};
    bool repeat_satellite{false};

    /* HS PCPS Acquisition configuration */
    std::string dump_filename;
    int64_t resampled_fs{0LL};
    float samples_per_ms{0.0};
    float pfa{0.0};
    float pfa2{0.0};
    float resampler_ratio{1.0};
    uint32_t sampled_ms{1U};
    uint32_t ms_per_code{1U};
    uint32_t samples_per_chip{2U};
    uint32_t chips_per_second{1023000U};
    uint32_t max_dwells{1U};
    uint32_t resampler_latency_samples{0U};
    uint32_t dump_channel{0U};
    int32_t doppler_min{-5000};
    bool bit_transition_flag{false};
    bool use_CFAR_algorithm_flag{true};
    bool dump{false};
    bool blocking{true};
    bool blocking_on_standby{false};  // enable it only for unit testing to avoid sample consume on idle status
    bool enable_monitor_output{false};
    bool enable_hs{false};

private:
    void SetDeviceFile();
    void SetDerivedParams();

    const std::string acquisition_device_name = "acquisition_S00_AXI";  // UIO device name
    const std::string PL_DDR4_RAM_device_name = "ddr4";
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQ_CONF_FPGA_H
