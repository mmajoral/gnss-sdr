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
    std::string PL_DDR4_device_name,
    uint32_t nsamples,
    uint32_t nsamples_first_block,
    uint32_t select_queue) : d_device_name(std::move(device_name)),
                             d_PL_DDR4_RAM_device_name(std::move(PL_DDR4_device_name)),
                             d_nsamples(nsamples),
                             d_nsamples_first_block(nsamples_first_block),
                             d_select_queue(select_queue)
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

volatile int16_t *Fpga_HS_Acquisition::open_PL_DDR4_RAM_device()
{
    // open communication with HW accelerator
    if ((d_fd_PL_DDR4_RAM = open(d_PL_DDR4_RAM_device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << d_PL_DDR4_RAM_device_name;
            std::cout << "Acq: cannot open deviceio" << d_PL_DDR4_RAM_device_name << '\n';
        }
    d_PL_DDR4_RAM_map_base = reinterpret_cast<volatile int16_t *>(mmap(nullptr, PL_DDR4_RAM_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, d_fd_PL_DDR4_RAM, 0));

    if (d_PL_DDR4_RAM_map_base == reinterpret_cast<void *>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA acquisition module into user memory";
            std::cout << "Acq: cannot map deviceio" << d_PL_DDR4_RAM_device_name << '\n';
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
    d_map_base[nsamples_first_block_reg_addr] = d_nsamples_first_block;
    uint32_t fpga_pl_ddr4_ram_addr_LSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_LSW);
    uint32_t fpga_pl_ddr4_ram_addr_MSW = (FPGA_PL_DDR4_RAM_ADDR & SELECT_MSW) >> SHIFT_32_BITS;
    d_map_base[write_address_LSW_reg_addr] = fpga_pl_ddr4_ram_addr_LSW;
    d_map_base[write_address_MSW_reg_addr] = fpga_pl_ddr4_ram_addr_MSW;
    d_map_base[enable_sanity_check_test_reg_addr] = 0;  // 1; // sanity check test
    d_map_base[int_on_rst_reg_addr] = 0;                // do not interrupt on reset
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
