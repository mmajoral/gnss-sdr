/*!
 * \file fpga_hs_acquisition.h
 * \brief Highly optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a highly optimized high sensitivity acquisition HW
 * accelerator in the FPGA
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_HS_ACQUISITION_H
#define GNSS_SDR_FPGA_HS_ACQUISITION_H

#include <cstdint>
#include <string>
#include <vector>  // for std::vector

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs
 * \{ */


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_HS_Acquisition
{
public:
    /*!
     * \brief Constructor
     */
    Fpga_HS_Acquisition(
        std::string device_name,
        uint32_t nsamples,
        uint32_t nsamples_first_block,
        uint32_t select_queue);

    /*!
     * \brief Destructor
     */
    ~Fpga_HS_Acquisition() = default;

    /*!
     * \brief Reset the FPGA PL.
     */
    void reset_acquisition();

    /*!
     * \brief stop the acquisition and the FPGA modules.
     */
    void stop_acquisition();

    /*!
     * \brief Write the acquisition parameters into the FPGA
     */
    void configure_acquisition();

    /*!
     * \brief capture samples
     */
    //std::vector<int16_t> * capture_samples();
    void capture_samples();

    /*!
     * \brief Open the device driver
     */
    void open_device();

    /*!
     * \brief Close the device driver
     */
    void close_device();

    /*!
     * \brief Open the PL DDR4 RAM memory device driver
     */
    int16_t *open_PL_DDR4_RAM_device();

    /*!
     * \brief Close the PL DDR4 RAM memory device driver
     */
    void close_PL_DDR4_RAM_device();

    /*!
     * \brief read the sample counter corresponding to the last sample capture
     */
    uint64_t read_sample_counter();

    /*!
     * \brief block the acquisition module
     */
    void block_acq();

    /*!
     * \brief unblock the acquisition module
     */
    void unblock_acq();


    //    void deb_unlock_acq();

private:
    // FPGA register adresses
    // write addresses
    static const uint32_t select_queue_reg_addr = 0;
    static const uint32_t blocking_reg_addr = 1;
    static const uint32_t nsamples_reg_addr = 2;
    static const uint32_t enable_sanity_check_test_reg_addr = 3;
    static const uint32_t sanity_check_test_counter_init_value_reg_addr = 4;
    static const uint32_t nsamples_first_block_reg_addr = 5;
    static const uint32_t reset_stop_start_acq_reg_addr = 8;
    static const uint32_t write_address_LSW_reg_addr = 12;
    static const uint32_t write_address_MSW_reg_addr = 13;
    static const uint32_t int_on_rst_reg_addr = 14;
    // read-write addresses
    static const uint32_t test_reg_addr = 15;
    // read addresses
    static const uint32_t result_valid_reg_addr = 0;
    static const uint32_t sample_counter_LSW_reg_addr = 1;
    static const uint32_t sample_counter_MSW_reg_addr = 2;
    static const uint32_t axi_error_flag_reg_addr = 14;
    static const uint32_t buffer_overflow_flag_reg_addr = 15;

    // FPGA register parameters
    static const uint32_t FPGA_PAGE_SIZE = 0x1000;             // default page size for the multicorrelator memory map
    static const uint32_t PL_DDR4_RAM_PAGE_SIZE = 0x20000000;  // PL DDR4 RAM page size
    static const uint32_t LAUNCH_ACQUISITION = 1;              // command to launch the acquisition process
    static const uint32_t RESET_ACQUISITION = 2;               // command to reset the acquisition and the FPGA Modules
    static const uint32_t STOP_ACQUISITION = 4;                // command to stop the acquisition and the FPGA modules
    static const uint32_t TEST_REG_SANITY_CHECK = 0x55AA;      // value to check the presence of the test register (to detect the hw)
    static const uint64_t SELECT_MSW = 0xFFFFFFFF00000000;     // select most significant 32 bits of a 64-bit address
    static const uint64_t SELECT_LSW = 0x00000000FFFFFFFF;     // select least significant 32 bits of a 64-bit address
    static const uint32_t SHIFT_32_BITS = 32;                  // 32-bit shift

    // PL DDR4 RAM address
    static const uint64_t FPGA_PL_DDR4_RAM_ADDR = 0x400000000;  // FPGA PL externalDDR4 RAM memory address
    // FPGA private functions
    void fpga_acquisition_test_register(void);
    void fpga_acquisition_test_PL_DDR4_RAM(void);

    std::string d_device_name;  // HW device name
    int64_t d_fs_in;

    volatile uint32_t *d_map_base;    // driver memory map
    int16_t *d_PL_DDR4_RAM_map_base;  // PL DDR4 RAM driver memory map
    int32_t d_fd;                     // ACQ IP driver descriptor
    int32_t d_fd_PL_DDR4_RAM;         // PL DDR4 RAM driver descriptor
    uint32_t *d_all_fft_codes;        // memory that contains all the code ffts
    uint32_t d_vector_length;         // number of samples including padding and number of ms
    uint32_t d_excludelimit;
    uint32_t d_nsamples_total;        // number of samples including padding
    uint32_t d_nsamples;              // number of samples not including padding
    uint32_t d_nsamples_first_block;  // number of samples of the first coherent integration
    uint32_t d_select_queue;          // queue selection
    uint32_t d_doppler_max;           // max doppler
    uint32_t d_doppler_step;          // doppler step
    uint32_t d_PRN;                   // PRN
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_HS_ACQUISITION_H
