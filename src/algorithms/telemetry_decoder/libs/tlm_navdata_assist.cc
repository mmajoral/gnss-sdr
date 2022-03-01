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

Tlm_navdata_assist::Tlm_navdata_assist(const Tlm_Conf &conf)
{
    navdata_assist_Tow = conf.navdata_assist_Tow;
    navdata_assist_samplestamp = conf.navdata_assist_samplestamp;
}

Tlm_navdata_assist::~Tlm_navdata_assist()
{

}
