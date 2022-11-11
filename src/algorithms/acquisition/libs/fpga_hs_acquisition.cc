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
    int32_t sampled_ms) : d_device_name(std::move(device_name)),
                          d_fs_in(fs_in),
                          d_nsamples(nsamples),
                          d_nsamples_first_block(nsamples_first_block),
                          d_select_queue(select_queue),
                          d_fft_size(fft_size)
{
    d_max_dwells = max_dwells;

    // High Sensitivity Acquisition IP sanity check
    Fpga_HS_Acquisition::open_device();
    Fpga_HS_Acquisition::reset_acquisition();
    Fpga_HS_Acquisition::fpga_acquisition_test_register();
    Fpga_HS_Acquisition::close_device();

    // PL DDR4 RAM sanity check
    Fpga_HS_Acquisition::open_PL_DDR4_RAM_device();
    Fpga_HS_Acquisition::fpga_acquisition_test_PL_DDR4_RAM();
    Fpga_HS_Acquisition::close_PL_DDR4_RAM_device();

    // compute xFFT hardware assistance twiddle factors
    compute_twiddle_factors();

    d_buffer_data = volk_gnsssdr::vector<std::complex<float>>(d_fft_size);

    d_xfft_num_channels = d_fft_size / FPGA_xFFT_SIZE;

    d_scaling_factor = (8.0 * (1e-4 / static_cast<float>(sampled_ms))) / (pow(2, FPGA_xFFT_NUM_BITS - 1));

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
}

int16_t *Fpga_HS_Acquisition::open_PL_DDR4_RAM_device()
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
    return d_PL_DDR4_RAM_map_base;
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
    d_map_base[nsamples_first_block_reg_addr] = d_nsamples;  //d_nsamples_first_block;
    uint32_t fpga_pl_ddr4_ram_addr_LSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[write_address_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[write_address_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;
}


void Fpga_HS_Acquisition::close_device()
{
    auto *aux = const_cast<uint32_t *>(d_map_base);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE) == -1)
        {
            std::cout << "Failed to unmap memory uio\n";
        }
    close(d_fd);
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
    uint32_t ncoh_integr_counter,
    uint32_t doppler_index)
{
    // configure Doppler Wipeoff
    float phase_step_rad_real = 2.0F * (doppler_freq) / static_cast<float>(d_fs_in);
    auto phase_step_rad_int = static_cast<int32_t>(phase_step_rad_real * (POW_2_31));
    d_map_base[phase_step_rad_Doppl_Wipeoff_reg_addr] = phase_step_rad_int;
    d_map_base[phase_step_rad_incr_Doppl_Wipeoff_reg_addr] = 0;

    // configure the number of samples
    d_map_base[nsamples_Doppl_Wipeoff_xFFT_reg_addr] = d_fft_size;

    // configure PL DDR4 read addresses
    uint32_t fpga_pl_ddr4_ram_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + d_fft_size * 4 * (ncoh_integr_counter - 1)) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;

    // configure log small FFT length and forward FFT
    d_map_base[fwd_inv_fft_length_reg_addr] = 0x30;  // disable doppl wipeoff=0x70 -- enable doppl wipeoff= 0x30;

    // configure PL DDR4 write addresses , here 3 is the number of doppler searches
    uint32_t fpga_pl_ddr4_ram_wr_addr_LSW = fpga_pl_ddr4_ram_addr_LSW + (d_fft_size)*7 * 4 + (d_fft_size)*4 * 7 * doppler_index;  // + (d_fft_size)*4*doppler_index;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_LSW;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;

    d_vect_addr = fpga_pl_ddr4_ram_wr_addr_LSW / 2;
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


void Fpga_HS_Acquisition::compute_twiddle_factors()
{
    d_fft_combine_twiddle_factors = volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>>(5, volk_gnsssdr::vector<std::complex<float>>(d_fft_size));
    d_ifft_combine_twiddle_factors = volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>>(5, volk_gnsssdr::vector<std::complex<float>>(d_fft_size));

    for (uint32_t index1 = 0; index1 < 5; index1++)
        {
            for (uint32_t index2 = 0; index2 < d_fft_size; index2++)
                {
                    d_fft_combine_twiddle_factors[index1][index2] = {
                        static_cast<float>(cos(((static_cast<float>(index1 + 1) * (static_cast<float>(index2)) / static_cast<float>(d_fft_size))) * 2.0 * M_PI)),
                        static_cast<float>(-sin(((static_cast<float>(index1 + 1) * (static_cast<float>(index2)) / static_cast<float>(d_fft_size))) * 2.0 * M_PI))};
                    d_ifft_combine_twiddle_factors[index1][index2] = {d_fft_combine_twiddle_factors[index1][index2].real(),
                        -d_fft_combine_twiddle_factors[index1][index2].imag()};
                }
        }
}


void Fpga_HS_Acquisition::run_Doppl_Wipeoff_FFT(void)
{
    run_Doppl_Wipeoff_xFFT();

    uint32_t xfft_status_data = d_map_base[xfft_status_data_reg_addr];
    int32_t block_exp[d_xfft_num_channels];
    d_max_block_exp_fft = 0;
    for (uint k = 0; k < d_xfft_num_channels; k++)
        {
            block_exp[k] = (xfft_status_data >> k * 5) & 0x1F;
            if (block_exp[k] > d_max_block_exp_fft)
                {
                    d_max_block_exp_fft = block_exp[k];
                }
        }

    volatile int16_t *vect_samples = static_cast<int16_t *>(d_PL_DDR4_RAM_map_base);

    for (uint32_t index2 = 0; index2 < FPGA_xFFT_SIZE; index2++)
        {
            std::complex<float> fft_value0 = {static_cast<int16_t>((vect_samples[d_vect_addr + (0 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (0 + index2 * d_xfft_num_channels) * 2 + 1]))};
            std::complex<float> fft_value1 = {static_cast<int16_t>((vect_samples[d_vect_addr + (1 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (1 + index2 * d_xfft_num_channels) * 2 + 1]))};
            std::complex<float> fft_value2 = {static_cast<int16_t>((vect_samples[d_vect_addr + (2 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (2 + index2 * d_xfft_num_channels) * 2 + 1]))};
            std::complex<float> fft_value3 = {static_cast<int16_t>((vect_samples[d_vect_addr + (3 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (3 + index2 * d_xfft_num_channels) * 2 + 1]))};
            std::complex<float> fft_value4 = {static_cast<int16_t>((vect_samples[d_vect_addr + (4 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (4 + index2 * d_xfft_num_channels) * 2 + 1]))};
            std::complex<float> fft_value5 = {static_cast<int16_t>((vect_samples[d_vect_addr + (5 + index2 * d_xfft_num_channels) * 2])),
                static_cast<int16_t>((vect_samples[d_vect_addr + (5 + index2 * d_xfft_num_channels) * 2 + 1]))};

            for (uint32_t index1 = 0; index1 < d_xfft_num_channels; index1++)
                {
                    d_buffer_data[(index1 * FPGA_xFFT_SIZE) + index2] = (fft_value0 + fft_value1 * d_fft_combine_twiddle_factors[0][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value2 * d_fft_combine_twiddle_factors[1][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value3 * d_fft_combine_twiddle_factors[2][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value4 * d_fft_combine_twiddle_factors[3][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value5 * d_fft_combine_twiddle_factors[4][(index1 * FPGA_xFFT_SIZE) + index2]);
                }
        }
}


void Fpga_HS_Acquisition::run_code_mult(volk_gnsssdr::vector<std::complex<float>> &d_fft_codes)
{
    volk_32fc_x2_multiply_32fc(d_buffer_data.data(), d_buffer_data.data(), d_fft_codes.data(), d_fft_size);
}


void Fpga_HS_Acquisition::configure_iFFT(uint32_t ncoh_integr_counter, uint32_t doppler_index)
{
    // set up addresses
    uint32_t fpga_pl_ddr4_ram_addr_LSW = ((FPGA_PL_DDR4_RAM_ADDR + d_fft_size * 4 * (ncoh_integr_counter - 1) + (d_fft_size)*7 * 4 + (d_fft_size)*4 * 7 * doppler_index + (d_fft_size)*4 * 7 * 10) & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_MSW) >> SHIFT_32_BITS;
    uint32_t fpga_pl_ddr4_ram_wr_addr_LSW = fpga_pl_ddr4_ram_addr_LSW + (d_fft_size)*4 * 7 * 10;  // assume max 10 doppler indices for now
    d_vect_addr = fpga_pl_ddr4_ram_addr_LSW / 2;
    d_vect_addr2 = fpga_pl_ddr4_ram_wr_addr_LSW / 2;

    // configure the number of samples
    d_map_base[nsamples_Doppl_Wipeoff_xFFT_reg_addr] = d_fft_size;

    // configure PL DDR4 read addresses
    d_map_base[read_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[read_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;

    // configure log small FFT length and forward FFT
    d_map_base[fwd_inv_fft_length_reg_addr] = 0x50;  // disable doppl wipeoff=0x50 -- enable doppl wipeoff= 0x10; //16; // FW FFTS

    // configure PL DDR4 write addresses , here 3 is the number of doppler searches
    uint32_t fpga_pl_ddr4_ram_wr_addr_LSW2 = fpga_pl_ddr4_ram_wr_addr_LSW;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_LSW_reg_addr] = fpga_pl_ddr4_ram_wr_addr_LSW2;
    d_map_base[write_address_Doppl_Wipeoff_xFFT_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;
}


void Fpga_HS_Acquisition::run_iFFT(volk_gnsssdr::vector<std::complex<float>> &buffer_short_ifft_data)
{
    volatile int16_t *vect_samples = static_cast<int16_t *>(d_PL_DDR4_RAM_map_base);

    // normalize input signal
    float max_val = 0, min_val = 0;
    for (uint32_t k = 0; k < d_fft_size; k++)
        {
            if (d_buffer_data[k].real() > max_val)
                {
                    max_val = d_buffer_data[k].real();
                }
            if (d_buffer_data[k].real() < min_val)
                {
                    min_val = d_buffer_data[k].real();
                }
            if (d_buffer_data[k].imag() > max_val)
                {
                    max_val = d_buffer_data[k].imag();
                }
            if (d_buffer_data[k].imag() < min_val)
                {
                    min_val = d_buffer_data[k].imag();
                }
        }
    if (min_val > max_val)
        {
            max_val = min_val;
        }

    // quantize the code mult and write it back to the PL DDR4
    for (uint32_t k = 0; k < d_fft_size; k++)
        {
            int16_t re_val = static_cast<int16_t>(32767.0 * d_buffer_data[k].real() / max_val);
            int16_t im_val = static_cast<int16_t>(32767.0 * d_buffer_data[k].imag() / max_val);
            vect_samples[d_vect_addr + k * 2] = re_val;
            vect_samples[d_vect_addr + k * 2 + 1] = im_val;
        }

    // run iFFT
    run_Doppl_Wipeoff_xFFT();

    uint32_t xfft_status_data = d_map_base[xfft_status_data_reg_addr];
    int32_t block_exp[d_xfft_num_channels];
    int32_t max_block_exp = 0;
    for (uint k = 0; k < d_xfft_num_channels; k++)
        {
            block_exp[k] = (xfft_status_data >> k * 5) & 0x1F;
            if (block_exp[k] > max_block_exp)
                {
                    max_block_exp = block_exp[k];
                }
        }

    float final_scaling_factor = max_val * d_scaling_factor * pow(2, d_max_block_exp_fft) * pow(2, max_block_exp);

    // combine using twiddle factors
    for (uint32_t index2 = 0; index2 < FPGA_xFFT_SIZE; index2++)
        {
            std::complex<float> fft_value0 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (0 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (0 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};
            std::complex<float> fft_value1 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (1 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (1 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};
            std::complex<float> fft_value2 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (2 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (2 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};
            std::complex<float> fft_value3 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (3 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (3 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};
            std::complex<float> fft_value4 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (4 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (4 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};
            std::complex<float> fft_value5 = {static_cast<int16_t>((vect_samples[d_vect_addr2 + (5 + index2 * d_xfft_num_channels) * 2])) * final_scaling_factor,
                static_cast<int16_t>((vect_samples[d_vect_addr2 + (5 + index2 * d_xfft_num_channels) * 2 + 1])) * final_scaling_factor};

            for (uint32_t index1 = 0; index1 < d_xfft_num_channels; index1++)
                {
                    buffer_short_ifft_data[(index1 * FPGA_xFFT_SIZE) + index2] = (fft_value0 + fft_value1 * d_ifft_combine_twiddle_factors[0][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value2 * d_ifft_combine_twiddle_factors[1][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value3 * d_ifft_combine_twiddle_factors[2][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value4 * d_ifft_combine_twiddle_factors[3][(index1 * FPGA_xFFT_SIZE) + index2] + fft_value5 * d_ifft_combine_twiddle_factors[4][(index1 * FPGA_xFFT_SIZE) + index2]);
                }
        }
}
