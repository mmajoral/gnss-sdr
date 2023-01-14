/*!
 * \file galileo_e5a_pcps_hs_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals for the FPGA high-sensitivity acquisition
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

#include "galileo_e5a_pcps_hs_acquisition_fpga.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

GalileoE5aPcpsHSAcquisitionFpga::GalileoE5aPcpsHSAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : gnss_synchro_(nullptr),
      role_(role),
      doppler_center_(0),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams),
      acq_pilot_(configuration->property(role + ".acquire_pilot", false)),
      acq_iq_(configuration->property(role + ".acquire_iq", false))
{
    acq_parameters_.SetFromHSConfiguration(configuration, role, fpga_downsampling_factor, fpga_buff_num, GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_CODE_LENGTH_CHIPS);

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
    code_ = volk_gnsssdr::vector<std::complex<float>>(vector_length_);

    sampled_ms_ = acq_parameters_.sampled_ms;

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


void GalileoE5aPcpsHSAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GalileoE5aPcpsHSAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GalileoE5aPcpsHSAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GalileoE5aPcpsHSAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GalileoE5aPcpsHSAcquisitionFpga::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_fpga_->set_doppler_center(doppler_center_);
}


void GalileoE5aPcpsHSAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE5aPcpsHSAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GalileoE5aPcpsHSAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GalileoE5aPcpsHSAcquisitionFpga::set_local_code()
{
    uint32_t num_codes = sampled_ms_ / (GALILEO_E5A_CODE_PERIOD_MS);  // code period in ms

    if (acq_iq_)
        {
            acq_pilot_ = false;
        }

    std::array<char, 3> signal_;
    signal_[0] = '5';
    signal_[2] = '\0';

    if (acq_iq_)
        {
            signal_[1] = 'X';
        }
    else if (acq_pilot_)
        {
            signal_[1] = 'Q';
        }
    else
        {
            signal_[1] = 'I';
        }

    galileo_e5_a_code_gen_complex_sampled(code_, gnss_synchro_->PRN, signal_, fs_in_, 0, num_codes);

    acquisition_fpga_->set_local_code(code_.data());
}


void GalileoE5aPcpsHSAcquisitionFpga::reset()
{
    acquisition_fpga_->set_active(true);
}


void GalileoE5aPcpsHSAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GalileoE5aPcpsHSAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GalileoE5aPcpsHSAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GalileoE5aPcpsHSAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GalileoE5aPcpsHSAcquisitionFpga::get_right_block()
{
    return nullptr;
}

uint64_t GalileoE5aPcpsHSAcquisitionFpga::get_sample_counter()
{
    return acquisition_fpga_->get_sample_counter();
}
