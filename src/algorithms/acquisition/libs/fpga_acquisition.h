/*!
 * \file fpga_acquisition.h
 * \brief Highly optimized FPGA vector correlator class
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          </ul>
 *
 * Class that controls and executes a highly optimized acquisition HW
 * accelerator in the FPGA
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2060  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FPGA_ACQUISITION_H
#define GNSS_SDR_FPGA_ACQUISITION_H

#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <complex>
#include <cstdint>
#include <string>
#include <utility>  // for std::move, std::pair
#include <vector>   // for std::vector

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs
 * \{ */


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Fpga_Acquisition
{
public:
    /*!
     * \brief Constructor
     */
    Fpga_Acquisition(
        std::string device_name,
        uint32_t select_queue,
        std::vector<std::pair<uint32_t, uint32_t>> &downsampling_filter_specs,
        uint32_t &max_FFT_size);

    /*!
     * \brief Destructor
     */
    ~Fpga_Acquisition() = default;

    /*!
     * \brief Initialize acquisition parameters
     */
    void init(uint32_t nsamples, uint32_t d_fft_size,
        int64_t resampled_fs, uint32_t downsampling_filter_num, uint32_t excludelimit, const volk_gnsssdr::vector<std::complex<float>> *prn_codes, uint32_t num_prns, bool enable_zero_padding);

    /*!
     * \brief Select the code with the chosen PRN
     */
    bool set_local_code(uint32_t PRN);

    /*!
     * \brief Configure the doppler sweep parameters in the FPGA
     */
    void set_doppler_sweep(uint32_t num_sweeps, uint32_t doppler_step, int32_t doppler_min);

    /*!
     * \brief Run the acquisition process in the FPGA
     */
    void run_acquisition();

    /*!
     * \brief Read the results of the acquisition process
     */
    void read_acquisition_results(
        uint32_t *max_index,
        float *firstpeak,
        float *secondpeak,
        uint64_t *initial_sample,
        float *power_sum,
        uint32_t *doppler_index,
        uint32_t *total_blk_exp);

    /*!
     * \brief Reset the FPGA PL.
     */
    void reset_acquisition();

    /*!
     * \brief stop the acquisition and the FPGA modules.
     */
    void stop_acquisition();

    /*!
     * \brief Set the block exponent of the FFT in the FPGA.
     */
    void set_block_exp(uint32_t total_block_exp);

    /*!
     * \brief Write the PRN code in the FPGA
     */
    void write_local_code(void);

    /*!
     * \brief Write the acquisition parameters into the FPGA
     */
    void configure_acquisition(void);

    /*!
     * \brief Open the device driver
     */
    void open_device();

    /*!
     * \brief Close the device driver
     */
    void close_device();

private:
    // FPGA register addresses

    // write-only registers
    static const uint32_t FREQ_BAND_DOWNSAMPLE_REG_ADDR = 0;      // Select frequency band and downsampling filter
    static const uint32_t FFT_LENGTH_REG_ADDR = 1;                // Length of the FFT
    static const uint32_t CORR_NSAMPLES_REG_ADDR = 2;             // Correlation length
    static const uint32_t DOPPLER_MIN_REG_ADDR = 3;               // Doppler min
    static const uint32_t DOPPLER_STEP_REG_ADDR = 4;              // Doppler step
    static const uint32_t NUM_DOPPLER_SEARCH_STEPS_REG_ADDR = 5;  // Number of Doppler search steps
    static const uint32_t PROG_MEM_ADDR = 6;                      // Access to the memory storing the PRN code of the target satellite.
    static const uint32_t LOG2_FFT_LENGTH_REG_ADDR = 7;           // Log2(FFT_LENGTH)
    static const uint32_t ACQ_COMMAND_FLAGS_REG_ADDR = 8;         // Flags that reset, start, and stop the acquisition process.
    static const uint32_t CLEAR_MEM_REG_ADDR = 9;                 // Flag that resets the write address of the PRN code memory.
    static const uint32_t MAX_FFT_SCALING_FACTOR_REG_ADDR = 11;   // Reference FFT scaling factor
    static const uint32_t EXCL_LIM_REG_ADDR = 12;                 // Exclude Limit value for the second FFT peak search process

    // read-write registers
    static const uint32_t TEST_REG_ADDR = 15;

    // read-only registers
    static const uint32_t RESULT_VALID_REG_ADDR = 0;                      // Flag that indicates a valid result
    static const uint32_t SAMPLESTAMP_LSW_REG_ADDR = 1;                   // Sample stamp LSW
    static const uint32_t SAMPLESTAMP_MSW_REG_ADDR = 2;                   // Sample stamp MSW
    static const uint32_t MAG_SQ_FIRST_PEAK_REG_ADDR = 3;                 // magnitude squared of the first peak
    static const uint32_t MAG_SQ_SECOND_PEAK_REG_ADDR = 4;                // magnitude squared of the second peak
    static const uint32_t ACQ_DELAY_SAMPLES_REG_ADDR = 5;                 // acquisition delay in samples
    static const uint32_t DOPPLER_INDEX_REG_ADDR = 7;                     // Doppler index
    static const uint32_t FFT_SCALING_FACTOR_REG_ADDR = 8;                // Scaling factor applied by the FFT
    static const uint32_t MAX_FFT_SIZE_REG_ADDR = 9;                      // Maximum FFT size supported by the FPGA
    static const uint32_t DOWNSAMPLING_FILTER_DEC_FACTORS_REG_ADDR = 10;  // Available decimation factors
    static const uint32_t DOWNSAMPLING_FILTER_LATENCIES_REG_ADDR = 11;    // Available downsampling filter latencies
    static const uint32_t FPGA_IP_CORE_VERSION_REG_ADDR = 14;             // FPGA acquisition IP core version

    // FPGA register parameters
    static const uint32_t FPGA_PAGE_SIZE = 0x1000;                // default page size for the multicorrelator memory map
    static const uint32_t LAUNCH_ACQUISITION = 1;                 // command to launch the acquisition process
    static const uint32_t RESET_ACQUISITION = 2;                  // command to reset the acquisition and the FPGA Modules
    static const uint32_t STOP_ACQUISITION = 4;                   // command to stop the acquisition and the FPGA modules
    static const uint32_t TEST_REG_SANITY_CHECK = 0x55AA;         // value to check the presence of the test register (to detect the hw)
    static const uint32_t LOCAL_CODE_CLEAR_MEM = 0x10000000;      // command to clear the internal memory of the multicorrelator
    static const uint32_t MEM_LOCAL_CODE_WR_ENABLE = 0x0C000000;  // command to enable the ENA and WR pins of the internal memory of the multicorrelator
    static const uint32_t POW_2_2 = 4;                            // 2^2 (used for the conversion of floating point numbers to integers)
    static const uint32_t POW_2_31 = 2147483648;                  // 2^31 (used for the conversion of floating point numbers to integers)
    static const uint32_t MAX_FILTERS_AVAILABLE = 2;              // maximum number of downsampling filters available in the FPGA by default
    static const uint32_t DEFAULT_MAX_FFT_SIZE = 32768;           // default maximum FFT size supported by the FPGA
    static const uint32_t ACQ_BUFF_0 = 0;                         // FPGA Acquisition IP buffer containing L1/E1 frequency band samples by default.
    static const uint32_t ACQ_BUFF_1 = 0;                         // FPGA Acquisition IP buffer containing L2 or L5/E5a frequency band samples by default.

    // bit manipulation
    static const uint32_t QUANT_BITS_LOCAL_CODE = 16;
    static const uint32_t SELECT_LSBITS = 0x0000FFFF;
    static const uint32_t SELECT_MSBITS = 0xFFFF0000;
    static const uint32_t SELECT_ALL_CODE_BITS = 0xFFFFFFFF;
    static const uint32_t SHL_CODE_BITS = 65536;
    static const uint32_t RSHIFT_4_BITS = 0x4;
    static const uint32_t RSHIFT_8_BITS = 0x8;
    static const uint32_t BIT_MASK_4 = 0xF;
    static const uint32_t BIT_MASK_8 = 0xFF;

    // Downsampling default constants
    const uint32_t DEFAULT_DOWNSAMPLING_FILTER_DELAY = 40;  // default downsampling filter delay (for FPGA Acquisition IP core versions earlier than v1.1)
    const uint32_t DEFAULT_DOWNSAMPLING_FACTOR = 4;         // default downsampling factor (for FPGA Acquisition IP core versions earlier than v1.1)

    // private methods
    void pack_fft_codes(uint32_t nsamples, uint32_t nsamples_total, uint32_t num_prns, std::vector<uint32_t> &all_fft_codes_packed, const volk_gnsssdr::vector<std::complex<float>> &prn_codes, bool enable_zero_padding, uint32_t IP_core_version);
    void fpga_acquisition_test_register(void);
    uint32_t read_ipcore_version();
    void set_downsampling_factor(std::vector<std::pair<uint32_t, uint32_t>> &downsampling_filter_specs, uint32_t &max_FFT_size, uint32_t IP_core_version);

    std::vector<std::pair<uint32_t, uint32_t>> d_downsampling_filter_specs;
    std::vector<uint32_t> d_all_fft_codes_packed;
    std::string d_device_name;      // HW device name
    int64_t d_resampled_fs;         // sampling frequency
    volatile uint32_t *d_map_base;  // driver memory map
    uint32_t d_IP_core_version;     // IP core version
    int32_t d_fd;                   // driver descriptor
    uint32_t d_fft_size;            // number of samples including padding
    uint32_t d_excludelimit;
    uint32_t d_nsamples;                   // number of samples not including padding
    uint32_t d_filter_num;                 // Selected downsampling filter
    uint32_t d_downsampling_factor;        // downsampling_factor
    uint32_t d_downsampling_filter_delay;  // Impulse response delay of the downsampling filter
    uint32_t d_select_queue;               // queue selection
    uint32_t d_PRN;                        // PRN
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FPGA_ACQUISITION_H
