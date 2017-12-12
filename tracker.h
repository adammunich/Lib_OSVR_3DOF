/**
* @file tracker.h
*
* @brief 3dof functions _H
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

	// Math
	#define twosCompliment(q, f) ((int32_t)(f * (float)(1 << q)))
	#define degreesToRadiansTwosComplimentQ14(f) (int32_t)((285.954745) * f)
	#define deciDegreesToRadiansTwosComplimentQ14(f) (int32_t)((28.5954745) * f)

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
		public:
		Tracker_(void);
		uint8_t begin(void);
		void end(void);
		size_t update(void);		
	};
	extern Tracker_ osvr_tracker;	

#endif
