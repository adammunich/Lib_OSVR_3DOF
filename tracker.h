/**
* @file tracker.h
*
* @brief OSVR 3dof functions _H
*
* @copyright
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* @copyright
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* @author Arduino LLC
* @author Adam Munich
*/
 
#ifndef _OSVR_3DOF_H_
	#define _OSVR_3DOF_H_

	#include <Arduino.h>
	#include "../Lib_Arduino_Default/HID/HID.h"

	#if !defined(_USING_HID)
		#error "Arduino variant does not have HID support"
	#endif
	
	// Math
	#define twosCompliment(q, f) ((int32_t)(f * (float)(1 << q)))
	#define degreesToRadiansTwosComplimentQ14(f) (int32_t)((285.954745) * f)
	#define deciDegreesToRadiansTwosComplimentQ14(f) (int32_t)((28.5954745) * f)
	#define degressToRadiansScalar ((float) 0.0174532925)

	// ----------------------------------------------------------------------------------
	// Handy quaternion integer struct
	typedef struct
	{
		int32_t i;
		int32_t j;
		int32_t k;
		int32_t real;
		int8_t q_point;
	} osvr_tracker_integer_quaternion_t;

	// ----------------------------------------------------------------------------------
	// Handy euler angles struct
	typedef struct
	{
		float x = 0;
		float y = 0;
		float z = 0;
	} osvr_tracker_float_euler_t;	

	//  Low level key report: up to 6 keys and shift, ctrl etc at once
	typedef struct
	{
		uint8_t vrpn_report_version;
		uint8_t vrpn_message_sequence_number;
		uint8_t vrpn_quaternion_i_lsb;
		uint8_t vrpn_quaternion_i_msb;
		uint8_t vrpn_quaternion_j_lsb;
		uint8_t vrpn_quaternion_j_msb;
		uint8_t vrpn_quaternion_k_lsb;
		uint8_t vrpn_quaternion_k_msb;
		uint8_t vrpn_quaternion_real_lsb;
		uint8_t vrpn_quaternion_real_msb;
		uint8_t vrpn_angular_velocity_about_x_lsb;
		uint8_t vrpn_angular_velocity_about_x_msb;
		uint8_t vrpn_angular_velocity_about_y_lsb;
		uint8_t vrpn_angular_velocity_about_y_msb;
		uint8_t vrpn_angular_velocity_about_z_lsb;
		uint8_t vrpn_angular_velocity_about_z_msb;
	} TrackerReport;

	class Tracker_
	{
		private:
		TrackerReport osvr_tracker_hid_report;
		void sendHIDReport(TrackerReport* report);
		void integer_math_convert_euler(osvr_tracker_float_euler_t euler, osvr_tracker_integer_quaternion_t * quaterinon, int8_t q_point);
		void integer_math_rotate_quaternion(osvr_tracker_integer_quaternion_t rotationVector, osvr_tracker_integer_quaternion_t * quaternion, int8_t q_point);
		
		public:
		Tracker_(void);
		uint8_t begin(void);
		void end(void);
		size_t update(void);		
	};
	extern Tracker_ osvr_tracker;	
		
	extern uint8_t g_osvr_tracker_use_magnetometer;
	extern uint8_t g_osvr_tracker_tare_now;
	extern uint8_t g_osvr_tracker_save_dcd_now;
	extern uint8_t g_osvr_tracker_enable_dcd_now;
	extern uint8_t g_osvr_tracker_clear_dcd_now;
		
#endif
