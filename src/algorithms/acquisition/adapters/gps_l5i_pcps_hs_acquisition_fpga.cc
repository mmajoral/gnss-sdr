/*!
 * \file gps_l5i_pcps_hs_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an Acquisition Interface for
 *  GPS L5i signals for the FPGA high-sensitivity acquisition
 * \author Marc Majoral, 2023. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_l5i_pcps_hs_acquisition_fpga.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include "gps_l5_signal_replica.h"
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

GpsL5iPcpsHSAcquisitionFpga::GpsL5iPcpsHSAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : gnss_synchro_(nullptr),
                                role_(role),
                                doppler_center_(0),
                                channel_(0),
                                doppler_step_(0),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    acq_parameters_.SetFromHSConfiguration(configuration, role, fpga_downsampling_factor, fpga_buff_num, GPS_L5I_CODE_RATE_CPS, GPS_L5I_CODE_LENGTH_CHIPS);

    LOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);

    fs_in_ = acq_parameters_.fs_in;

    code_length_ = acq_parameters_.code_length;
    vector_length_ = static_cast<unsigned int>(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0));
    sampled_ms_ = acq_parameters_.sampled_ms;

    // pre-compute all PRN codes
    uint32_t fft_size;
    if (acq_parameters_.sampled_ms == acq_parameters_.ms_per_code)
        {
            fft_size = vector_length_;
        }
    else
        {
            fft_size = vector_length_ * 2;
        }
    uint32_t num_codes = sampled_ms_ / (GPS_L5I_PERIOD_S * ms_per_s);  // code period in ms
    codes_ = volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>>(NUM_PRNs, volk_gnsssdr::vector<std::complex<float>>(fft_size));

    volk_gnsssdr::vector<std::complex<float>> code_aux(vector_length_);
    auto fft_if = gnss_fft_fwd_make_unique(fft_size);  // Direct FFT

    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l5i_code_gen_complex_sampled(code_aux, PRN, fs_in_, num_codes);

            // COD
            // Here we want to create a buffer that looks like this:
            // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
            // where c_i is the local code and there are L zeros and L chips
            if (acq_parameters_.bit_transition_flag)
                {
                    const int32_t offset = fft_size / 2;
                    std::fill_n(fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
                    std::copy(code_aux.data(), code_aux.data() + offset, fft_if->get_inbuf() + offset);
                }
            else
                {
                    if (acq_parameters_.sampled_ms == acq_parameters_.ms_per_code)
                        {
                            std::copy(code_aux.data(), code_aux.data() + vector_length_, fft_if->get_inbuf());
                        }
                    else
                        {
                            std::fill_n(fft_if->get_inbuf(), fft_size - vector_length_, gr_complex(0.0, 0.0));
                            std::copy(code_aux.data(), code_aux.data() + vector_length_, fft_if->get_inbuf() + vector_length_);
                        }
                }

            fft_if->execute();  // We need the FFT of local code
            volk_32fc_conjugate_32fc(codes_[PRN - 1].data(), fft_if->get_outbuf(), fft_size);
        }

    acquisition_fpga_ = pcps_make_hs_acquisition_fpga(acq_parameters_);

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GpsL5iPcpsHSAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GpsL5iPcpsHSAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GpsL5iPcpsHSAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GpsL5iPcpsHSAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GpsL5iPcpsHSAcquisitionFpga::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_fpga_->set_doppler_center(doppler_center_);
}


void GpsL5iPcpsHSAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL5iPcpsHSAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GpsL5iPcpsHSAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GpsL5iPcpsHSAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code(codes_[(gnss_synchro_->PRN) - 1]);
}


void GpsL5iPcpsHSAcquisitionFpga::reset()
{
    acquisition_fpga_->set_active(true);
}


void GpsL5iPcpsHSAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GpsL5iPcpsHSAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GpsL5iPcpsHSAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GpsL5iPcpsHSAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GpsL5iPcpsHSAcquisitionFpga::get_right_block()
{
    return nullptr;
}

uint64_t GpsL5iPcpsHSAcquisitionFpga::get_sample_counter()
{
    return acquisition_fpga_->get_sample_counter();
}
