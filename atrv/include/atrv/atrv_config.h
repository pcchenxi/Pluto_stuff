/*  ATRV constants
 *  Modified by John Folkesson from ATRVJR from
 *  B21 Driver - By David Lu!! 2/2010
 *  Modified from Player code
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ATRV_CONFIG_H
#define ATRV_CONFIG_H

// Odometery Constants  CAlIBRATION NEEDED
// ===================
// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 143214
//was 90810
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 37781
//was 37000

// Sonar Constants CAlIBRATION NEEDED
// ===============
// Arbitrary units per meter (for sonar)
#define RANGE_CONVERSION 1476
#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 3000


//#define SONAR_ECHO_DELAY 30000
//#define SONAR_PING_DELAY 10000
//#define SONAR_SET_DELAY  10000



const int SONARS_PER_BANK[] = {4, 4, 4 };
#define SONAR_BANKS = 3
#define SONAR_MAX_PER_BANK 4
#define SONAR_COUNT 12

//x y  z (m) and theta degrees per sonar  guesssed about

const double SONARS_X[] = { -0.335, -0.425, -0.425,  -0.335, 0.425,  0.425, 0.425,  0.335, 0.425, 0.425,  0.425,  0.335 };

const double SONARS_Y[] = { 0.225, 0.175, -0.175, -0.225, 0.065, 0.11, 0.165, 0.225, -0.065, -0.11, -0.165, -0.225 };

const double SONARS_Z[] = { 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39 };

const double SONARS_THETA[] = { 90, 180, 180, -90, 0, 15, 30, 90, 0, -15, -30, -90 };

// Digital IO constants
// ====================

#define BUMPER_COUNT 2
const double BUMPER_X[] = {0.585, -0.585};
const double BUMPER_Y[] = {0, 0};
const double BUMPER_Z[] = {0.28, 0.28};

// Misc Constants
// ==============
#define USE_JOYSTICK 0
#define POWER_OFFSET 1.2
#define PLUGGED_THRESHOLD 25.0


#define HOME_BEARING -32500


#endif









