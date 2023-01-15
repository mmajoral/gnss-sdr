/*!
 * \file gps_l1_ca_pcps_hs_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals for the FPGA high-sensitivity acquisition
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_HS_ACQUISITION_FPGA_H
#define GNSS_SDR_GPS_L1_CA_PCPS_HS_ACQUISITION_FPGA_H

#include "acq_conf_fpga.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_hs_acquisition_fpga.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <memory>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block off-loaded on an FPGA
 * to an AcquisitionInterface for GPS L1 C/A signals
 */
class GpsL1CaPcpsHSAcquisitionFpga : public AcquisitionInterface
{
public:
    /*!
     * \brief Constructor
     */
    GpsL1CaPcpsHSAcquisitionFpga(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Destructor
     */
    ~GpsL1CaPcpsHSAcquisitionFpga() = default;

    /*!
     * \brief Role
     */
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_Acquisition_Fpga"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_Acquisition_Fpga";
    }

    /*!
     * \brief Returns size of lv_16sc_t
     */
    inline size_t item_size() override
    {
        return sizeof(int16_t);
    }

    /*!
     * \brief Connect
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block
     */
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     * tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Set acquisition channel unique ID
     */
    inline void set_channel(unsigned int channel) override
    {
        channel_ = channel;
        acquisition_fpga_->set_channel(channel_);
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        channel_fsm_ = channel_fsm;
        acquisition_fpga_->set_channel_fsm(channel_fsm);
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    std::string item_type_;

    void set_threshold(float threshold) override;

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max) override;

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step) override;

    /*!
     * \brief Set Doppler center for the grid search
     */
    void set_doppler_center(int doppler_center) override;

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for GPS L1/CA PCPS acquisition algorithm.
     */
    void set_local_code() override;

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag() override;

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset() override;

    /*!
     * \brief If state = 1, it forces the block to start acquiring from the first sample
     */
    void set_state(int state) override;

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    /*!
     * \brief Set Resampler Latency
     */
    void set_resampler_latency(uint32_t latency_samples __attribute__((unused))) override{};

    /*!
     * \brief Get the value of the sample counter
     */
    uint64_t get_sample_counter();

private:
    static const uint32_t NUM_PRNs = 32;

    static const uint32_t fpga_downsampling_factor = 4;  // downampling factor in the FPGA
    static const uint32_t fpga_buff_num = 0;             // L1/E1 band

    pcps_hs_acquisition_fpga_sptr acquisition_fpga_;
    volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>> codes_;
    std::weak_ptr<ChannelFsm> channel_fsm_;
    Gnss_Synchro* gnss_synchro_;
    Acq_Conf_Fpga acq_parameters_;
    std::string role_;
    int64_t fs_in_;
    int32_t doppler_center_;
    unsigned int vector_length_;
    unsigned int code_length_;
    uint32_t channel_;
    uint32_t doppler_max_;
    uint32_t doppler_step_;
    unsigned int sampled_ms_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_PCPS_HS_ACQUISITION_FPGA_H
