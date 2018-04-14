/**
* @file tracker.h
*
* @brief OSVR 3DOF Functions _H
*
* @copyright
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation; either
* version 3.0 of the License, or (at your option) any later version.
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
 
#ifndef _VRPN_3DOF_H_
	#define _VRPN_3DOF_H_

	#include <Arduino.h>
	//#include <HID.h>
	//#include "HID.h"
	#include "../Lib_Arduino_Default/HID/HID.h"

	#if !defined(_USING_HID)
		#error "Arduino variant does not have HID support"
	#endif
	
	// Math
	#define vrpn_twosCompliment(q, f) ((int32_t)(f * (float)(1 << q)))
	#define vrpn_degreesToRadiansTwosComplimentQ14(f) (int32_t)((285.954745) * f)
	#define vrpn_deciDegreesToRadiansTwosComplimentQ14(f) (int32_t)((28.5954745) * f)
	#define vrpn_degressToRadiansScalar ((float) 0.0174532925)

	// ----------------------------------------------------------------------------------
	// Handy quaternion integer struct
	typedef struct
	{
		int32_t i;
		int32_t j;
		int32_t k;
		int32_t real;
		int8_t q_point;
	} vrpnTracker_int_quaternion_t;

	// ----------------------------------------------------------------------------------
	// Handy euler angles struct
	typedef struct
	{
		float x = 0;
		float y = 0;
		float z = 0;
	} vrpnTracker_float_euler_t;	
	
	// ----------------------------------------------------------------------------------	
	typedef enum class vrpnTracker_sensorType
	{
		SENSOR_FUSION_NO_MAGNETOMETER,
		SENSOR_FUSION_WITH_MAGNETOMETER,
		SIMPLE_ORIENTATION,
		SIMPLE_GRAVITY_REFERENCED_ORIENTATION,
		SIMPLE_GEOMAGNETIC_REFERENCED_ORIENTATION,
		FILTERED_ORIENTATION,
		FILTERED_GRAVITY_REFERENCED_ORIENTATION
	} vrpnTracker_sensorType_t;	
	
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
	} vrpnTrackerReport;

	class vrpnTracker_
	{
		private:
		vrpnTrackerReport osvr_tracker_hid_report;
		void sendHIDReport(vrpnTrackerReport* report);
		void integer_math_convert_euler(vrpnTracker_float_euler_t euler, vrpnTracker_int_quaternion_t * quaterinon, int8_t q_point);
		void integer_math_rotate_quaternion(vrpnTracker_int_quaternion_t rotationVector, vrpnTracker_int_quaternion_t * quaternion, int8_t q_point);
		
		public:
		vrpnTracker_(void);
		uint8_t begin(uint32_t pin_rstn, uint32_t pin_bootn, uint32_t pin_intn);
		void setup(vrpnTracker_sensorType_t sensor_type, uint16_t reportInterval_us, float prediction_amount_s, float euler_x, float euler_y, float euler_z);
		void end(void);
		size_t loopTask(void);		
	};
	extern vrpnTracker_ vrpn_tracker;	
		
	extern uint8_t g_vrpn_tracker_use_magnetometer;
	extern uint8_t g_vrpn_tracker_tare_now;
	extern uint8_t g_vrpn_tracker_save_dcd_now;
	extern uint8_t g_vrpn_tracker_enable_dcd_now;
	extern uint8_t g_vrpn_tracker_clear_dcd_now;
		
#endif
