/*!
 * \file fpga_hs_acquisition.cc
 * \brief Highly optimized FPGA high sensitivity vector correlator class
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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fpga_hs_acquisition.h"
#include "MATH_CONSTANTS.h"  // for TWO_PI
#include <glog/logging.h>    // for LOG
#include <cmath>             // for log2
#include <fcntl.h>           // libraries used by the GIPO
#include <iostream>          // for operator<<
#include <sys/mman.h>        // libraries used by the GIPO
#include <unistd.h>          // for write, close, read, ssize_t
#include <utility>           // for move

// DEBUG TEST SW MODEL OF HW MODEL
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <math.h>
#include <unistd.h>  // FOR SLEEPING

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(exp)              \
    ({                                       \
        decltype(exp) _rc;                   \
        do                                   \
            {                                \
                _rc = (exp);                 \
            }                                \
        while (_rc == -1 && errno == EINTR); \
        _rc;                                 \
    })
#endif


Fpga_HS_Acquisition::Fpga_HS_Acquisition(std::string device_name,
    int64_t fs_in,
    uint32_t nsamples,
    uint32_t nsamples_first_block,
    uint32_t select_queue,
    uint32_t fft_size,
    uint32_t max_dwells,
    bool sort_ifft_output) : d_device_name(std::move(device_name)),
                             d_fs_in(fs_in),
                             d_nsamples(nsamples),
                             d_nsamples_first_block(nsamples_first_block),
                             d_select_queue(select_queue),
                             d_fft_size(fft_size),
                             d_max_dwells(max_dwells),
                             d_sort_ifft_output(sort_ifft_output)
{
    // High Sensitivity Acquisition IP sanity check
    Fpga_HS_Acquisition::open_device();
    Fpga_HS_Acquisition::reset_acquisition();
    Fpga_HS_Acquisition::fpga_acquisition_test_register();
    Fpga_HS_Acquisition::close_device();

    // PL DDR4 RAM sanity check
    Fpga_HS_Acquisition::open_PL_DDR4_RAM_device();
    Fpga_HS_Acquisition::fpga_acquisition_test_PL_DDR4_RAM();
    Fpga_HS_Acquisition::close_PL_DDR4_RAM_device();

    d_buffer_data = volk_gnsssdr::vector<std::complex<float>>(d_fft_size);

    d_xfft_num_channels = d_fft_size / FPGA_xFFT_SIZE;

    DLOG(INFO) << "Acquisition HS FPGA class created";
}


void Fpga_HS_Acquisition::open_device()
{
    // open communication with HW accelerator
    if ((d_fd = open(d_device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_device_name;
            std::cout << "Acq: cannot open deviceio" << d_device_name << '\n';
        }
    d_map_base = reinterpret_cast<volatile uint32_t *>(mmap(nullptr, FPGA_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd, 0));

    if (d_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map deviceio" << d_device_name << '\n';
        }

    // open the PL DDR4 RAM memory device as well as it is used in all cases
    Fpga_HS_Acquisition::open_PL_DDR4_RAM_device();
}

void Fpga_HS_Acquisition::open_PL_DDR4_RAM_device()
{
    // open communication with HW accelerator
    if ((d_fd_PL_DDR4_RAM = open("/dev/mem", O_RDWR)) == -1)
        {
            LOG(WARNING) << "Cannot open PL DDR4 RAM device";
            std::cout << "Acq: cannot open PL DDR4 RAM device" << '\n';
        }

    d_PL_DDR4_RAM_map_base = reinterpret_cast<int16_t *>(mmap(nullptr, PL_DDR4_RAM_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd_PL_DDR4_RAM, FPGA_PL_DDR4_RAM_ADDR));

    if (d_PL_DDR4_RAM_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map the FPGA acquisition module into user memory" << '\n';
        }
}

void Fpga_HS_Acquisition::open_PL_DDR4_RAM_LC_device()
{
    // open communication with HW accelerator
    if ((d_fd_PL_DDR4_RAM_LC = open("/dev/mem", O_RDWR)) == -1)
        {
            LOG(WARNING) << "Cannot open PL DDR4 RAM device";
            std::cout << "Acq: cannot open PL DDR4 RAM device" << '\n';
        }

    d_PL_DDR4_RAM_LC_map_base = reinterpret_cast<int16_t *>(mmap(nullptr, PL_DDR4_RAM_LC_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd_PL_DDR4_RAM_LC, FPGA_PL_DDR4_RAM_LC_ADDR));

    if (d_PL_DDR4_RAM_LC_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map the FPGA acquisition module into user memory" << '\n';
        }
}

void Fpga_HS_Acquisition::fpga_acquisition_test_register()
{
    // sanity check : check test register
    const uint32_t writeval = TEST_REG_SANITY_CHECK;

    // write value to test register
    d_map_base[test_reg_addr] = writeval;

    // read value from test register
    const uint32_t readval = d_map_base[test_reg_addr];

    if (writeval != readval)
        {
            LOG(WARNING) << "Acquisition test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
        }
}

void Fpga_HS_Acquisition::fpga_acquisition_test_PL_DDR4_RAM()
{
    // PL DDR4 RAM sanity check
    // quickly test the PL DDR4 RAM by writing and reading from various addresses along the PL DDR4 RAM memory map
    uint32_t readval;
    bool test_ok = true;
    for (uint32_t k = 0; k < 8; k++)
        {
            d_PL_DDR4_RAM_map_base[k * 16777216] = k;
            readval = d_PL_DDR4_RAM_map_base[k * 16777216];
            if (k != readval)
                {
                    test_ok = false;
                }
        }
    if (!test_ok)
        {
            LOG(WARNING) << "Acquisition test PL DDR4 RAM sanity check failed";
        }
    else
        {
            LOG(INFO) << "Acquisition test register sanity check success!";
        }
}

void Fpga_HS_Acquisition::capture_samples()
{
    // enable interrupts
    int32_t reenable = 1;
    const ssize_t nbytes = TEMP_FAILURE_RETRY(write(d_fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error enabling run in the FPGA.\n";
        }

    // lauch sample capture process
    d_map_base[reset_stop_start_acq_reg_addr] = LAUNCH_ACQUISITION;
    int32_t irq_count;

    // wait for interrupt
    const ssize_t nb = read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            std::cout << "acquisition module Read failed to retrieve 4 bytes!\n";
            std::cout << "acquisition module Interrupt number " << irq_count << '\n';
        }
}

void Fpga_HS_Acquisition::read_samples(uint32_t ncoh_integr_counter, volk_gnsssdr::vector<std::complex<float>> &input_signal)
{
    volatile int16_t *vect_samples = static_cast<int16_t *>(d_PL_DDR4_RAM_map_base);
    for (uint32_t k = 0; k < d_nsamples_first_block; k++)
        {
            input_signal[k] = std::complex<float>(vect_samples[2 * k + (ncoh_integr_counter * d_nsamples_first_block * 2)], vect_samples[(2 * k) + 1 + (ncoh_integr_counter * d_nsamples_first_block * 2)]);
        }
}

uint64_t Fpga_HS_Acquisition::read_sample_counter()
{
    uint32_t readval = d_map_base[sample_counter_LSW_reg_addr];  // read sample counter (LSW)
    uint64_t initial_sample_tmp = static_cast<uint64_t>(readval);

    uint64_t readval_long = d_map_base[sample_counter_MSW_reg_addr];  // read sample counter (MSW)
    uint64_t readval_long_shifted = readval_long << 32;               // 2^32

    initial_sample_tmp += readval_long_shifted;  // 2^32
    return initial_sample_tmp;
}

void Fpga_HS_Acquisition::configure_acquisition()
{
    d_map_base[select_queue_reg_addr] = d_select_queue;
    d_map_base[nsamples_reg_addr] = d_nsamples;
    d_map_base[nsamples_first_block_reg_addr] = d_nsamples_first_block;
    uint32_t fpga_pl_ddr4_ram_addr_LSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[write_address_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[write_address_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;
}

void Fpga_HS_Acquisition::set_local_code(volk_gnsssdr::vector<std::complex<float>> &fft_code)
{
    // find maximum value
    float max_val = 0;
    for (uint32_t k = 0; k < d_fft_size; k++)
        {
            if (abs(fft_code[k].real()) > max_val)
                {
                    max_val = abs(fft_code[k].real());
                }
            else if (abs(fft_code[k].imag()) > max_val)
                {
                    max_val = abs(fft_code[k].imag());
                }
        }

    // a separate memory region is used for the local code. The local code memory region is not catched.
    // the local code DDR4 RAM memory addresses are only used when FPGA HW acceleration is enabled
    open_PL_DDR4_RAM_LC_device();

    uint32_t k = 0;
    for (uint32_t index1 = 0; index1 < FPGA_xFFT_SIZE; index1++)
        {
            for (uint32_t index2 = 0; index2 < d_xfft_num_channels; index2++)
                {
                    // normalize local code by maximum value. Use the same sample ordering as as the FPGA FFT output values
                    float real_part = round(fft_code[index2 * FPGA_xFFT_SIZE + index1].real() * (MAX_POS_VALUE_16BIT) / max_val);
                    float imag_part = round(fft_code[index2 * FPGA_xFFT_SIZE + index1].imag() * (MAX_POS_VALUE_16BIT) / max_val);
                    // write local code to DDR4 RAM memory
                    d_PL_DDR4_RAM_LC_map_base[2 * k] = static_cast<int16_t>(real_part);
                    d_PL_DDR4_RAM_LC_map_base[2 * k + 1] = static_cast<int16_t>(imag_part);
                    k++;
                }
        }

    close_PL_DDR4_RAM_LC_device();
}

void Fpga_HS_Acquisition::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd);
    // close the PL DDR4 RAM memory device as well as it is used in all cases
    Fpga_HS_Acquisition::close_PL_DDR4_RAM_device();
}

void Fpga_HS_Acquisition::close_PL_DDR4_RAM_device()
{
    auto *aux = const_cast<int16_t *>(d_PL_DDR4_RAM_map_base);
    if (munmap(static_cast<void *>(aux), PL_DDR4_RAM_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd_PL_DDR4_RAM);
}

void Fpga_HS_Acquisition::close_PL_DDR4_RAM_LC_device()
{
    auto *aux = const_cast<int16_t *>(d_PL_DDR4_RAM_LC_map_base);
    if (munmap(static_cast<void *>(aux), PL_DDR4_RAM_LC_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd_PL_DDR4_RAM_LC);
}


void Fpga_HS_Acquisition::reset_acquisition()
{
    d_map_base[reset_stop_start_acq_reg_addr] = RESET_ACQUISITION;  // setting bit 2 of d_map_base[8] resets the acquisition. This causes a reset of all
                                                                    // the FPGA HW modules including the multicorrelators
}


void Fpga_HS_Acquisition::stop_acquisition()
{
    d_map_base[reset_stop_start_acq_reg_addr] = STOP_ACQUISITION;  // setting bit 3 of d_map_base[8] stops the acquisition module. This stops all
                                                                   // the FPGA HW modules including the multicorrelators
    // unblock the acquisition module
    d_map_base[blocking_reg_addr] = 0;
}


void Fpga_HS_Acquisition::block_acq()
{
    d_map_base[blocking_reg_addr] = 1;
}

void Fpga_HS_Acquisition::unblock_acq()
{
    d_map_base[blocking_reg_addr] = 0;
}

void Fpga_HS_Acquisition::configure_Doppl_Wipeoff_FFT(float doppler_freq,
    //uint32_t ncoh_integr_counter,
    //uint32_t doppler_index,
    uint32_t offset_rd_addr,
    uint32_t offset_wr_addr)
{
    // configure Doppler Wipeoff
    float phase_step_rad_real = 2.0F * (doppler_freq) / static_cast<float>(d_fs_in);
    auto phase_step_rad_int = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));
    d_map_base[phase_step_rad_Doppl_Wipeoff_reg_addr] = phase_step_rad_int;
    d_map_base[phase_step_rad_incr_Doppl_Wipeoff_reg_addr] = 0;

    // configure the number of samples
    d_map_base[nsamples_Doppl_Wipeoff_xFFT_reg_addr] = d_fft_size;

    // configure PL DDR4 read addresses
    uint32_t fpga_pl_ddr4_ram_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_rd_addr) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_rd_addr) & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;

    // configure log small FFT length and forward FFT
    d_map_base[fwd_inv_fft_length_reg_addr] = REORDER_xFFT_COMBINING_RESULT | FW_FFT | FPGA_LOG2_xFFT_SIZE;

    // configure PL DDR4 write addresses , here 3 is the number of doppler searches
    uint32_t fpga_pl_ddr4_ram_wr_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_wr_addr) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_wr_addr_MSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_wr_addr) & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_LSW;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_MSW;

    // local code read address
    uint32_t offset_lc_rd_addr = FPGA_PL_DDR4_RAM_LC_OFFSET_ADDR;
    d_map_base[local_code_read_address_LSW_reg_addr] = ((FPGA_PL_DDR4_RAM_ADDR + offset_lc_rd_addr) & SELECT_LSW);
    d_map_base[local_code_read_address_MSW_reg_addr] = ((FPGA_PL_DDR4_RAM_ADDR + offset_lc_rd_addr) & SELECT_MSW) >> SHIFT_32_BITS;

    // output scaling factors
    d_map_base[output_scaling_factors_reg_addr] = (FFT_OUTPUT_SCALING_FACTOR << xFFT_OUTPUT_SCALING_FACTOR_BIT_POS) + CODE_MULT_OUTPUT_SCALING_FACTOR;
}

void Fpga_HS_Acquisition::run_Doppl_Wipeoff_xFFT()
{
    // enable interrupts
    int32_t reenable = 1;
    const ssize_t nbytes = TEMP_FAILURE_RETRY(write(d_fd, reinterpret_cast<void *>(&reenable), sizeof(int32_t)));
    if (nbytes != sizeof(int32_t))
        {
            std::cerr << "Error enabling run in the FPGA.\n";
        }

    // lauch Doppler Wipeoff and/or xFFT
    d_map_base[reset_stop_start_acq_reg_addr] = LAUNCH_DOPPL_WIPEOFF_XFFT;
    int32_t irq_count;

    // wait for interrupt
    const ssize_t nb = read(d_fd, &irq_count, sizeof(irq_count));
    if (nb != sizeof(irq_count))
        {
            std::cout << "acquisition module Read failed to retrieve 4 bytes!\n";
            std::cout << "acquisition module Interrupt number " << irq_count << '\n';
        }
}


void Fpga_HS_Acquisition::run_Doppl_Wipeoff_FFT(float &scaling_factor_fft)
{
    // run Doppler wipeoff + FFT + local code mult
    run_Doppl_Wipeoff_xFFT();

    // read FFT scaling factor
    scaling_factor_fft = d_map_base[xfft_status_data_reg_addr];
}

void Fpga_HS_Acquisition::configure_iFFT(uint32_t offset_rd_addr, uint32_t offset_wr_addr)
{
    // set up read addresses
    uint32_t fpga_pl_ddr4_ram_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_rd_addr) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_rd_addr) & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;

    // configure the number of samples
    d_map_base[nsamples_Doppl_Wipeoff_xFFT_reg_addr] = d_fft_size;

    // configure log small FFT length and forward FFT
    if (d_sort_ifft_output)
        {
            d_map_base[fwd_inv_fft_length_reg_addr] = REORDER_xFFT_COMBINING_RESULT | DISABLE_CODE_MULT | DISABLE_DOPPLER_WIPEOFF | FPGA_LOG2_xFFT_SIZE;
        }
    else
        {
            d_map_base[fwd_inv_fft_length_reg_addr] = DISABLE_CODE_MULT | DISABLE_DOPPLER_WIPEOFF | FPGA_LOG2_xFFT_SIZE;
        }

    // set up write addresses
    uint32_t fpga_pl_ddr4_ram_wr_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_wr_addr) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_wr_addr_MSW = ((FPGA_PL_DDR4_RAM_ADDR + offset_wr_addr) & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_LSW;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_MSW;

    // output scaling factor
    d_map_base[output_scaling_factors_reg_addr] = (IFFT_OUTPUT_SCALING_FACTOR << xFFT_OUTPUT_SCALING_FACTOR_BIT_POS);
}

void Fpga_HS_Acquisition::run_iFFT(float &scaling_factor_ifft)
{
    // run iFFT
    run_Doppl_Wipeoff_xFFT();

    // read iFFT scaling factor
    scaling_factor_ifft = d_map_base[xfft_status_data_reg_addr];
}

void Fpga_HS_Acquisition::apply_scaling_correction_factor(lv_32fc_t *input_buff, float scaling_factor, uint32_t offset_rd_addr)
{
    uint32_t vect_addr = offset_rd_addr / 2;  // 16-bit addressing mode
    volatile int16_t *vect_samples = static_cast<int16_t *>(d_PL_DDR4_RAM_map_base);

    float *aPtr = (float *)input_buff;

    // read the iFFT results
    for (uint32_t k = 0; k < d_fft_size; k++)
        {
            *aPtr++ = vect_samples[vect_addr + 2 * k] * scaling_factor;      // re part
            *aPtr++ = vect_samples[vect_addr + 2 * k + 1] * scaling_factor;  // im part
        }
}

uint32_t Fpga_HS_Acquisition::invert_ifft_ordering(uint32_t indext)
{
    // invert the ordering of the ifft output
    return (indext % FPGA_xFFT_NUM_CHAN) * FPGA_xFFT_SIZE + (indext / FPGA_xFFT_NUM_CHAN);  // integer division rounds towards 0
}

void Fpga_HS_Acquisition::run_coherent_integration(float doppler_freq, uint32_t ncoh_integr_counter, uint32_t doppler_index, lv_32fc_t *buffer_short_ifft_data)
{
    float scaling_factor_fft, scaling_factor_ifft;

    // set input and output memory addresses for Doppler wipeoff, FFT and code mult
    uint32_t offset_rd_addr = d_fft_size * BYTES_PER_COMPLEX_SAMPLE * (ncoh_integr_counter - 1);
    uint32_t offset_wr_addr = offset_rd_addr + (d_fft_size)*d_max_dwells * BYTES_PER_COMPLEX_SAMPLE + (d_fft_size)*BYTES_PER_COMPLEX_SAMPLE * doppler_index;

    // configure Doppler wipeoff, FFT and code mult
    Fpga_HS_Acquisition::configure_Doppl_Wipeoff_FFT(doppler_freq, offset_rd_addr, offset_wr_addr);

    // run Doppler Wipeoff and FFT
    Fpga_HS_Acquisition::run_Doppl_Wipeoff_FFT(scaling_factor_fft);

    // set input and output memory addresses for the iFFT
    offset_rd_addr = offset_wr_addr;  // read from the output of the Doppler wipeoff, FFT and code mult
    offset_wr_addr = offset_rd_addr + (d_fft_size)*BYTES_PER_COMPLEX_SAMPLE * d_max_dwells;

    // configure iFFT
    Fpga_HS_Acquisition::configure_iFFT(offset_rd_addr, offset_wr_addr);

    // compute the inverse FFT
    Fpga_HS_Acquisition::run_iFFT(scaling_factor_ifft);

    // compute scaling factor
    float scaling_factor = SCALING_FACT_PREVENT_OVERFLOW * scaling_factor_ifft * scaling_factor_fft;

    // apply scaling factor and read final results
    Fpga_HS_Acquisition::apply_scaling_correction_factor(buffer_short_ifft_data, scaling_factor, offset_wr_addr);
}
