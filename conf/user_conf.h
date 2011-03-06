/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file
*
* 	@brief General Configuration
*
* 	These settings are general and valid across boards.
*
**/

#ifndef _USER_CONF_H_
#define _USER_CONF_H_

// System types
#define PX_GENERIC              1
#define PX_AIRFRAME_FIXED_WING  2
#define PX_AIRFRAME_QUADROTOR   3
#define PX_AIRFRAME_HELICOPTER  4
#define PX_AIRFRAME_COAXIAL     5
#define PX_GROUND_CAR           6

// Please select a system type
// Valid system types are listed above.
#define PX_VEHICLE_TYPE PX_GROUND_CAR


#if (PX_VEHICLE_TYPE < 1) | (PX_VEHICLE_TYPE > 6)
// If you get this error, you selected an invalid system type or misspelled
// the system type name.
#error INVALID SYSTEM TYPE, NOT FOUND IN LIST - ABORTING COMPILATION
#endif

#if !(defined PX_VEHICLE_TYPE)
// Uncomment the #define PX_VEHICLE_TYPE line above and select system type
// to resolve this error
#error PLEASE SELECT SYSTEM TYPE IN conf/user_conf.h FIRST! UNCOMMENT LINE define PX_VEHICLE_TYPE  - ABORTING COMPILATION
#endif

#endif /* _USER_CONF_H_ */
