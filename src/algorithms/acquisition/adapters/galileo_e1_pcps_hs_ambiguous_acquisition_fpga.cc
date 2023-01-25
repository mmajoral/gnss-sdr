/*!
 * \file galileo_e1_pcps_hs_ambiguous_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals for the FPGA high-sensitivity acquisition
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

#include "galileo_e1_pcps_hs_ambiguous_acquisition_fpga.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <gnuradio/fft/fft.h>     // for fft_complex
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

GalileoE1PcpsHSAmbiguousAcquisitionFpga::GalileoE1PcpsHSAmbiguousAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : gnss_synchro_(nullptr),
                                configuration_(configuration),
                                role_(role),
                                doppler_center_(0),
                                channel_(0),
                                doppler_step_(0),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    acq_parameters_.ms_per_code = 4;
    acq_parameters_.SetFromHSConfiguration(configuration_, role, fpga_downsampling_factor, fpga_buff_num, GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS);

    DLOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);

    fs_in_ = acq_parameters_.fs_in;

    acquire_pilot_ = configuration->property(role + ".acquire_pilot", false);

    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
    vector_length_ = static_cast<unsigned int>(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0));

    sampled_ms_ = acq_parameters_.sampled_ms;

    enable_hs_ = acq_parameters_.enable_hs;

    if (acq_parameters_.sampled_ms == acq_parameters_.ms_per_code)
        {
            fft_size_ = vector_length_;
        }
    else
        {
            if (enable_hs_)
                {
                    fft_size_ = vector_length_;
                }
            else
                {
                    fft_size_ = vector_length_ * 2;
                }
        }

    code_aux_ = volk_gnsssdr::vector<std::complex<float>>(vector_length_);
    code_ = volk_gnsssdr::vector<std::complex<float>>(fft_size_);
    fft_if_ = gnss_fft_fwd_make_unique(fft_size_);

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


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;
    acquisition_fpga_->set_doppler_center(doppler_center_);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE1PcpsHSAmbiguousAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_local_code()
{
    bool cboc = configuration_->property(
        "Acquisition" + std::to_string(channel_) + ".cboc", false);

    uint32_t num_codes;  //, codelength;
    if (enable_hs_)
        {
            // when using high sensitivity mode, the concatenated
            // PRN codes have to be interpolated for the whole
            // sample_ms_ duration, to prevent performance loss
            // when using sampling periods that are not a divider
            // of the PRN code duration.
            num_codes = sampled_ms_ / GALILEO_E1_CODE_PERIOD_MS;
            //codelength = vector_length_;
        }
    else
        {
            num_codes = 1;
            //codelength = code_length_;
        }
    //volk_gnsssdr::vector<std::complex<float>> code(codelength);

    if (acquire_pilot_ == true)
        {
            // set local signal generator to Galileo E1 pilot component (1C)
            std::array<char, 3> pilot_signal = {{'1', 'C', '\0'}};
            galileo_e1_code_gen_complex_sampled(code_aux_, pilot_signal,
                //cboc, gnss_synchro_->PRN, fs_in_, 0, num_codes, doppler_center_, false);
                cboc, gnss_synchro_->PRN, fs_in_, 0, num_codes, 0, false);
        }
    else
        {
            std::array<char, 3> Signal_{};
            Signal_[0] = gnss_synchro_->Signal[0];
            Signal_[1] = gnss_synchro_->Signal[1];
            Signal_[2] = '\0';
            galileo_e1_code_gen_complex_sampled(code_aux_, Signal_,
                cboc, gnss_synchro_->PRN, fs_in_, 0, num_codes, false);
        }

    own::span<gr_complex> code_span(code_.data(), vector_length_);

    if (enable_hs_)
        {
            for (unsigned int i = 0; i < sampled_ms_ / GALILEO_E1_CODE_PERIOD_MS; i++)
                {
                    uint32_t initial_sample = floorf(static_cast<float>((acq_parameters_.resampled_fs) * i * GALILEO_E1_CODE_PERIOD_MS) / 1000.0);
                    uint32_t end_sample = floorf(static_cast<float>((acq_parameters_.resampled_fs) * (i + 1) * GALILEO_E1_CODE_PERIOD_MS) / 1000.0);
                    for (unsigned int j = initial_sample; j < end_sample; j++)
                        {
                            if (GALILEO_E1_C_SECONDARY_CODE[i] == '0')
                                {
                                    code_aux_[j] = code_aux_[j];
                                }
                            else
                                {
                                    code_aux_[j] = -code_aux_[j];
                                }
                        }
                }
        }
    else
        {
            for (unsigned int i = 0; i < sampled_ms_ / 4; i++)
                {
                    std::copy_n(code_aux_.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
                }
        }

    // COD
    // Here we want to create a buffer that looks like this:
    // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
    // where c_i is the local code and there are L zeros and L chips
    if (acq_parameters_.bit_transition_flag)
        {
            const int32_t offset = fft_size_ / 2;
            std::fill_n(fft_if_->get_inbuf(), offset, gr_complex(0.0, 0.0));
            std::copy(code_aux_.data(), code_aux_.data() + offset, fft_if_->get_inbuf() + offset);
        }
    else
        {
            if (acq_parameters_.sampled_ms == acq_parameters_.ms_per_code)
                {
                    std::copy(code_aux_.data(), code_aux_.data() + vector_length_, fft_if_->get_inbuf());
                }
            else
                {
                    if (enable_hs_)
                        {
                            std::copy(code_aux_.data(), code_aux_.data() + vector_length_, fft_if_->get_inbuf());
                        }
                    else
                        {
                            std::fill_n(fft_if_->get_inbuf(), fft_size_ - vector_length_, gr_complex(0.0, 0.0));
                            std::copy(code_aux_.data(), code_aux_.data() + vector_length_, fft_if_->get_inbuf() + vector_length_);
                        }
                }
        }

    fft_if_->execute();  // We need the FFT of local code
    volk_32fc_conjugate_32fc(code_.data(), fft_if_->get_outbuf(), fft_size_);


    acquisition_fpga_->set_local_code(code_);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::reset()
{
    // This command starts the acquisition process
    acquisition_fpga_->set_active(true);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GalileoE1PcpsHSAmbiguousAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GalileoE1PcpsHSAmbiguousAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GalileoE1PcpsHSAmbiguousAcquisitionFpga::get_right_block()
{
    return nullptr;
}

uint64_t GalileoE1PcpsHSAmbiguousAcquisitionFpga::get_sample_counter()
{
    return acquisition_fpga_->get_sample_counter();
}
