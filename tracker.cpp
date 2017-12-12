/**
* @file tracker.cpp
*
* @brief 3dof functions
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

#include "./tracker.h"
#include "./sh2_arduino.h"
#include "../pin_settings.h"

// ----------------------------------------------------------------------------------
// Variables
// ----------------------------------------------------------------------------------

// Sensor object
Hillcrest_ g_hillcrest;

// Sensor report values
sh2_SensorValue_t g_osvr_tracker_sensor_values;

// Sensor orientation
sh2_Quaternion_2Compliment_t g_osvr_tracker_sensor_rotation;

	
// ----------------------------------------------------------------------------------
// Function definitions
// ----------------------------------------------------------------------------------			

// ----------------------------------------------------------------------------------
// Public	
						 	
Tracker_::Tracker_(void)
{
}

uint8_t Tracker_ :: begin(void)
{
	osvr_tracker_hid_report.vrpn_message_sequence_number = 0;
	
	// Sensor pins
	sh2_SensorPins_t osvr_tracker_sensor_pins;
	osvr_tracker_sensor_pins.pin_rstn = PIN_osvr_tracker_rstn;
	osvr_tracker_sensor_pins.pin_bootn = PIN_osvr_tracker_bootn;
	osvr_tracker_sensor_pins.pin_intn = PIN_osvr_tracker_intn;
		
	// Sensor type
	sh2_SensorType_t osvr_tracker_sensor_type = SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_NO_MAG;
	
	// Interrupt config
	sh2_SensorInterrupts_t osvr_tracker_sensor_ints;
	osvr_tracker_sensor_ints.report_interval_microseconds = 1500000;
	
	// DMP config
	sh2_SensorDMPConfig_t osvr_tracker_sensor_dmp;
	osvr_tracker_sensor_dmp.GIRV_PRED_AMT_S = 0.002;
	
	// Orientation config
		/* BROKEN */
	sh2_Euler_t sensorOrientation;
	sensorOrientation.x = 0;
	sensorOrientation.y = 0;
	sensorOrientation.z = 0;
		
	g_hillcrest.begin(osvr_tracker_sensor_pins);		
	g_hillcrest.configure(osvr_tracker_sensor_type, osvr_tracker_sensor_ints, osvr_tracker_sensor_dmp, sensorOrientation);
		
		/* HOTFIX */
	sh2_Euler_t rotation_euler;
						
	rotation_euler.x = -90;
	rotation_euler.y = 0;
	rotation_euler.z = 0;
		
	g_hillcrest.integer_math_convert_euler(rotation_euler, &g_osvr_tracker_sensor_rotation, 14);
}

void Tracker_ :: end(void)
{
	Hillcrest.end();
}
						
size_t Tracker_ :: update(void)
{												
	uint8_t osvr_tracker_sensor_event = g_hillcrest.checkReport(&g_osvr_tracker_sensor_values);
		
	if(osvr_tracker_sensor_event){
			
		Hillcrest.debugProdIds();	
		Hillcrest.debugValues(&g_osvr_tracker_sensor_values);
					
		sh2_Quaternion_2Compliment_t sensor_vector;
			
		sensor_vector.q_point = 14;
		sensor_vector.real = twosCompliment(14, g_osvr_tracker_sensor_values.un.gameRotationVector.real);
		sensor_vector.i	= twosCompliment(14, g_osvr_tracker_sensor_values.un.gameRotationVector.i);
		sensor_vector.j = twosCompliment(14, g_osvr_tracker_sensor_values.un.gameRotationVector.j);
		sensor_vector.k = twosCompliment(14, g_osvr_tracker_sensor_values.un.gameRotationVector.k);
			
		Hillcrest.integer_math_rotate_quaternion(g_osvr_tracker_sensor_rotation, &sensor_vector, 14);
			
		int16_t quat_real_q = sensor_vector.real;
		int16_t quat_i_q = sensor_vector.i;
		int16_t quat_j_q = sensor_vector.j;
		int16_t quat_k_q = sensor_vector.k;			 
								
		int16_t ang_vel_x = deciDegreesToRadiansTwosComplimentQ14(g_osvr_tracker_sensor_values.un.gyroIntegratedRV.angVelX);
		int16_t ang_vel_y = deciDegreesToRadiansTwosComplimentQ14(g_osvr_tracker_sensor_values.un.gyroIntegratedRV.angVelY);
		int16_t ang_vel_z = deciDegreesToRadiansTwosComplimentQ14(g_osvr_tracker_sensor_values.un.gyroIntegratedRV.angVelZ);
				
		osvr_tracker_hid_report.vrpn_report_version					= 0x02;
		osvr_tracker_hid_report.vrpn_message_sequence_number++;
		osvr_tracker_hid_report.vrpn_quaternion_i_lsb				= lowByte(quat_i_q);
		osvr_tracker_hid_report.vrpn_quaternion_i_msb				= highByte(quat_i_q);
		osvr_tracker_hid_report.vrpn_quaternion_j_lsb				= lowByte(quat_j_q);
		osvr_tracker_hid_report.vrpn_quaternion_j_msb				= highByte(quat_j_q);
		osvr_tracker_hid_report.vrpn_quaternion_k_lsb				= lowByte(quat_k_q);
		osvr_tracker_hid_report.vrpn_quaternion_k_msb				= highByte(quat_k_q);
		osvr_tracker_hid_report.vrpn_quaternion_real_lsb			= lowByte(quat_real_q);
		osvr_tracker_hid_report.vrpn_quaternion_real_msb			= highByte(quat_real_q);
		
		osvr_tracker_hid_report.vrpn_angular_velocity_about_x_lsb	= lowByte(ang_vel_x);
		osvr_tracker_hid_report.vrpn_angular_velocity_about_x_msb	= highByte(ang_vel_x);
		osvr_tracker_hid_report.vrpn_angular_velocity_about_y_lsb	= lowByte(ang_vel_y);
		osvr_tracker_hid_report.vrpn_angular_velocity_about_y_msb	= highByte(ang_vel_y);
		osvr_tracker_hid_report.vrpn_angular_velocity_about_z_lsb	= lowByte(ang_vel_z);
		osvr_tracker_hid_report.vrpn_angular_velocity_about_z_msb	= highByte(ang_vel_z);
	}
		
	return osvr_tracker_sensor_event;
}

Tracker_ osvr_tracker;