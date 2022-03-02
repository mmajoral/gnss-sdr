/*!
 * \file tlm_navdata_assist.cc
 * \brief Class that provides telemetry data assistance
 * \author Marc Majoral, 2021. mmajoral(at)cttc.es
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

#include "tlm_navdata_assist.h"
#include <cassert>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>

Tlm_navdata_assist::Tlm_navdata_assist(const Tlm_Conf &conf)
{
    navdata_assist_real_time = conf.navdata_assist_real_time;
    navdata_assist_Tow_ms = conf.navdata_assist_Tow_ms;
    navdata_assist_samplestamp = conf.navdata_assist_samplestamp;

    current_TOW_ms = 0;
}

uint32_t Tlm_navdata_assist::get_TOW_at_current_symbol_ms(uint32_t d_PRN_code_period_ms)
{
    current_TOW_ms = current_TOW_ms + d_PRN_code_period_ms;
    return current_TOW_ms;
}

uint32_t Tlm_navdata_assist::compute_elapsed_days(std::string dayofweek)
{
    if (dayofweek == "Mon")
        return 1;
    else if (dayofweek == "Tue")
        return 2;
    else if (dayofweek == "Wed")
        return 3;
    else if (dayofweek == "Thu")
        return 4;
    else if (dayofweek == "Fri")
        return 5;
    else if (dayofweek == "Sat")
        return 6;
    else
        return 0;
}

uint32_t Tlm_navdata_assist::compute_current_TOW(uint64_t Tracking_sample_counter, uint64_t fs)
{
    if (navdata_assist_real_time)
        {
            // 1 get current time date in timepoint
            std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();

            // 2 convert current time date in timepoint to date and time
            std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(pt.time_since_epoch());
            std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(ms);
            std::time_t tim = sec.count();
            std::size_t fractional_seconds = ms.count() % 1000;

            // std::localtime is not thread safe so we do these computations by hand
            char buffer[32];
            std::strncpy(buffer, std::ctime(&tim), 26);
            std::string dayofweek(buffer, 3);
            std::string current_hour(buffer + 11, 2);
            int current_hour_dec = std::stoi(current_hour, nullptr);
            std::string current_min(buffer + 14, 2);
            int current_min_dec = std::stoi(current_min, nullptr);
            std::string current_s(buffer + 17, 2);
            int current_s_dec = std::stoi(current_s, nullptr);

            // 3 compute seconds since start of week
            int days_since_start_of_week = compute_elapsed_days(dayofweek);
            int seconds_since_start_of_week = (days_since_start_of_week)*24 * 60 * 60 + current_hour_dec * 60 * 60 + current_min_dec * 60 + current_s_dec;

            current_TOW_ms = seconds_since_start_of_week * 1000;
            return current_TOW_ms;
        }
    else
        {
            current_TOW_ms = navdata_assist_Tow_ms + (Tracking_sample_counter - navdata_assist_samplestamp) * 1000 / fs;
            return current_TOW_ms;
        }
}

Tlm_navdata_assist::~Tlm_navdata_assist()
{
}
