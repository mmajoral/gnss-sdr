/*!
 * \file acq_conf_fpga.cc
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

#include "acq_conf_fpga.h"
#include "item_type_helpers.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <cmath>
#include <iostream>

void Acq_Conf_Fpga::SetFromConfiguration(const ConfigurationInterface *configuration,
    const std::string &role, uint32_t downs_factor, uint32_t sel_queue_fpga, uint32_t blk_exp, double chip_rate, double code_length_chips)
{
    // sampling frequency
    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    // max doppler
    doppler_max = configuration->property(role + ".doppler_max", doppler_max);

    // downsampling factor
    uint32_t downsampling_factor = configuration->property(role + ".downsampling_factor", downs_factor);
    fs_in = fs_in / downsampling_factor;

    // code length in samples
    code_length = static_cast<uint32_t>(std::round(static_cast<double>(fs_in) / (chip_rate / code_length_chips)));

    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f(static_cast<float>(code_length) * 2.0F));
    samples_per_code = pow(2, nbits);

    // repeat satellite
    repeat_satellite = configuration->property(role + ".repeat_satellite", false);

    // FPGA buffer number
    select_queue_Fpga = configuration->property(role + ".select_queue_Fpga", sel_queue_fpga);

    // exclusion limit
    excludelimit = static_cast<unsigned int>(1 + ceil((1.0 / chip_rate) * static_cast<float>(fs_in)));

    // acquisition step 2 parameters
    num_doppler_bins_step2 = configuration->property(role + ".second_nbins", num_doppler_bins_step2);
    doppler_step2 = configuration->property(role + ".second_doppler_step", doppler_step2);
    doppler_step = configuration->property(role + ".doppler_step", doppler_step);
    make_2_steps = configuration->property(role + ".make_two_steps", make_2_steps);
    max_num_acqs = configuration->property(role + ".max_num_acqs", 2);

    // reference for the FPGA FFT-IFFT attenuation factor
    total_block_exp = configuration->property(role + ".total_block_exp", blk_exp);

    SetDeviceFile();
}

void Acq_Conf_Fpga::SetFromHSConfiguration(const ConfigurationInterface *configuration,
    const std::string &role, uint32_t downs_factor, uint32_t sel_queue_fpga, double chip_rate, double code_length_chips)
{
    chips_per_second = chip_rate;

    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    // downsampling factor
    downsampling_factor = configuration->property(role + ".downsampling_factor", downs_factor);
    fs_in = fs_in / downsampling_factor;

    // code length in samples
    code_length = static_cast<uint32_t>(std::round(static_cast<double>(fs_in) / (chip_rate / code_length_chips)));

    doppler_max = configuration->property(role + ".doppler_max", doppler_max);
    sampled_ms = configuration->property(role + ".coherent_integration_time_ms", sampled_ms);
    bit_transition_flag = configuration->property(role + ".bit_transition_flag", bit_transition_flag);
    max_dwells = configuration->property(role + ".max_dwells", max_dwells);
    dump = configuration->property(role + ".dump", dump);
    dump_channel = configuration->property(role + ".dump_channel", dump_channel);
    blocking = configuration->property(role + ".blocking", blocking);
    dump_filename = configuration->property(role + ".dump_filename", dump_filename);

    if ((sampled_ms % ms_per_code) != 0)
        {
            LOG(WARNING) << "Parameter coherent_integration_time_ms should be a multiple of "
                         << ms_per_code << ". Setting it to " << ms_per_code;
            sampled_ms = ms_per_code;
        }

    resampled_fs = fs_in;

    num_doppler_bins_step2 = configuration->property(role + ".second_nbins", num_doppler_bins_step2);
    doppler_step2 = configuration->property(role + ".second_doppler_step", doppler_step2);
    doppler_step = configuration->property(role + ".doppler_step", doppler_step);
    pfa = configuration->property(role + ".pfa", pfa);
    if ((pfa < 0.0) or (pfa > 1.0))
        {
            LOG(WARNING) << "Parameter pfa should between 0.0 and 1.0. Setting it to 0.0";
            pfa = 0.0;
        }
    pfa2 = configuration->property(role + ".pfa_second_step", pfa2);
    if ((pfa2 <= 0.0) or (pfa2 > 1.0))
        {
            pfa2 = pfa;
        }
    make_2_steps = configuration->property(role + ".make_two_steps", make_2_steps);
    max_num_acqs = configuration->property(role + ".max_num_acqs", 2);
    blocking_on_standby = configuration->property(role + ".blocking_on_standby", blocking_on_standby);

    if (pfa <= 0.0)
        {
            // if pfa is not set, we use the first_vs_second_peak_statistic metric
            use_CFAR_algorithm_flag = false;
        }

    // repeat satellite
    repeat_satellite = configuration->property(role + ".repeat_satellite", false);

    // FPGA buffer number
    select_queue_Fpga = configuration->property(role + ".select_queue_Fpga", sel_queue_fpga);

    enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);

    enable_hs = configuration->property(role + ".enable_hs", false);

    SetDerivedParams();

    SetDeviceFile();
}


void Acq_Conf_Fpga::SetDerivedParams()
{
    samples_per_ms = static_cast<float>(resampled_fs) * 0.001F;
    samples_per_chip = static_cast<unsigned int>(std::ceil(static_cast<float>(resampled_fs) / chips_per_second));
    samples_per_code = samples_per_ms * ms_per_code;
}

void Acq_Conf_Fpga::SetDeviceFile()
{
    // UIO device file
    std::string device_io_name;
    // find the uio device file corresponding to the acquisition
    if (find_uio_dev_file_name(device_io_name, acquisition_device_name, 0) < 0)
        {
            std::cout << "Cannot find the FPGA uio device file corresponding to device name " << acquisition_device_name << std::endl;
            throw std::exception();
        }
    device_name = device_io_name;
}
