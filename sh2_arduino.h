/**
* @file sh2_arduino.h
*
* @brief Hillcrest Sensorhub Arduino Library _H
*
* @copyright
* Apache 2.0
*
* @copyright
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* @author Adam Munich
*/

#ifndef SH2_ARDUINO_H_
	#define SH2_ARDUINO_H_

	#include <Arduino.h>
	#include <stdbool.h>
	#include <Wire.h>

	#include "./bno080-driver/sh2.h"
	#include "./bno080-driver/sh2_hal.h"
	#include "./bno080-driver/shtp.h"
	#include "./bno080-driver/sh2_err.h"
	#include "./bno080-driver/sh2_SensorValue.h"	

	#define SH2_i2c_addr_dfu_0 (0x28)
	#define SH2_i2c_addr_dfu_1 (0x29)
	#define SH2_i2c_addr_0 (0x4A)
	#define SH2_i2c_addr_1 (0x4B)

	#define serial_out SerialUSB

	#define SH2_dfu_delay			(200) // [mS]
	#define SH2_reset_delay			(10) // [mS]
	#define SH2_max_events			(16)
	#define SH2_SHTP_header_length	(4)
	
	#ifndef ARRAY_LEN
		#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
	#endif

	#define twos_compliment(n, x) ((int32_t)(x * (float)(1 << n)))

	const float scaleDegToRad = 3.14159265358 / 180.0;
	

	// ----------------------------------------------------------------------------------
	// Interrupt flags 
	typedef enum
	{
		EVT_INTN,
		EVT_NONE
	} EventId_t;

	typedef struct
	{
		uint32_t t_ms;
		EventId_t id;
	} Event_t;

	// ----------------------------------------------------------------------------------
	// Sensor types
	typedef enum
	{
		SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_NO_MAG,
		SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_WITH_MAG,
		SH2_TYPE_ROTATION_VECTOR,
		SH2_TYPE_GAME_ROTATION_VECTOR,
		SH2_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
		SH2_TYPE_AR_VR_STABILIZED_ROTATION_VECTOR,
		SH2_TYPE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR			
	} sh2_SensorType_t;

	// ----------------------------------------------------------------------------------
	// Gyro integrated rotation vector configuration
	typedef struct
	{
		// Synchronization Interval
		uint32_t SH2_GIRV_SYNC_INTERVAL_US = 10000;
		// Maximum error
		float SH2_GIRV_MAX_ERR = 30.0;
		// Prediction Amount
		float GIRV_PRED_AMT_S = 0; //0.028; 
		// Alpha
		float SH2_GRIV_ALPHA = 0.303072543909142;
		// Beta
		float SH2_GRIV_BETA = 0.113295896384921;
		// Gamma
		float SH2_GRIV_GAMMA = 0.002776219713054;
	} sh2_SensorDMPConfig_t;

	// ----------------------------------------------------------------------------------
	// Pin configuration struct
	typedef struct
	{
		uint32_t pin_bootn;
		uint32_t pin_intn;	
		uint32_t pin_rstn;
	} sh2_SensorPins_t;

	// ----------------------------------------------------------------------------------
	// Handy euler angles 
	typedef struct
	{
		float x = 0;
		float y = 0;
		float z = 0;
	} sh2_Euler_t;

	// ----------------------------------------------------------------------------------
	// Handy quaternion integer struct
	typedef struct
	{
		int32_t i;
		int32_t j; 
		int32_t k;
		int32_t real;
		int8_t q_point;
	} sh2_Quaternion_2Compliment_t;

	// ----------------------------------------------------------------------------------
	// Interrupt configuration struct
	typedef struct
	{
		bool enable_reports_on_change = false;
		bool enable_change_relativity = false;         // Change sensitivity - true if relative; false if absolute
		bool enable_wakeup = false;
		bool enable_always_on = true;
		uint16_t change_threshold = 0;
		uint32_t report_interval_microseconds = 10000;
	} sh2_SensorInterrupts_t;

	// ----------------------------------------------------------------------------------
	// DCD configuration struct
	typedef struct
	{
		bool enable_accel_dcd = true;
		bool enable_gyro_dcd = true;
		bool enable_mag_dcd = true;		
	} sh2_SensorDCDConfig_t;

	// ----------------------------------------------------------------------------------
	// Class members
	class Hillcrest_
	{
		private:
		void sh2_hal_init(void);
		void sh2_i2cReset(void);
		void sh2_onReset(void);
		void sh2_configure(void);
		void sh2_startReports(void);
		void sh2_halTask(void);
		void sh2_parseEvent(const sh2_SensorEvent_t * event, sh2_SensorValue_t * values);
		void math_rotate(sh2_Euler_t euler, sh2_Quaternion_t * quaterinon);
		
		public:
		Hillcrest_(void);
		int8_t begin(sh2_SensorPins_t setupPins);
		void end(void);		
		void configure(sh2_SensorType_t sensorType, sh2_SensorInterrupts_t sensorInts, sh2_SensorDMPConfig_t sensorDMP, sh2_Euler_t sensorOrientation);
		void startReports(void);
		bool checkReport(sh2_SensorValue_t * values);		
		
		int8_t tare(bool tare_x, bool tare_y, bool tare_z, sh2_TareBasis_t basis, bool perist);
		int8_t saveDCD(void);
		int8_t setDCD(bool accel, bool gyro, bool mag);
		int8_t clearDCD(void);

		void debugValues(sh2_SensorValue_t * values);	
		void debugProdIds(void);
		
		void integer_math_convert_euler(sh2_Euler_t euler, sh2_Quaternion_2Compliment_t * quaterinon, int8_t q_point);
		void integer_math_rotate_quaternion(sh2_Quaternion_2Compliment_t rotationVector, sh2_Quaternion_2Compliment_t * quaternion, int8_t q_point);
	};
	extern Hillcrest_ Hillcrest;

#endif /* SH2_ARDUINO_H */