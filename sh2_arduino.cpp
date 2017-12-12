/**
* @file sh2_arduino.cpp
*
* @brief Hillcrest Sensorhub Arduino Library
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

// ----------------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------------
#include "./sh2_arduino.h"

#include <Wire.h>

#ifndef TwoWire_h
	#error "No I2C bus??"
#endif
	
// ----------------------------------------------------------------------------------
// Global variables 
// ----------------------------------------------------------------------------------
sh2_SensorEvent_t g_sh2_sensorEvent;
Event_t g_sh2_imu_event;

volatile bool g_sh2_resetPerformed = false;
volatile bool g_sh2_sensorReceived = false;
volatile bool g_sh2_startedReports = false;
volatile bool g_sh2_i2cResetNeeded = false;

sh2_SensorPins_t g_sh2_sensorPins;				// Pins struct
sh2_SensorType_t g_sh2_sensorType;				// Sensor type
sh2_SensorConfig_t g_sh2_sensorReportConfig;	// Report config struct
sh2_SensorDMPConfig_t g_sh2_settings_DMP;		// Settings - DMP

sh2_Quaternion_t g_sh2_settings_sensorOrientation;	// Settings - Sensor Orientation (broken?)
sh2_SensorDCDConfig_t g_sh2_sensorDCDConfig;	// Settings - DCD



// ----------------------------------------------------------------------------------
// State struct
typedef struct
{
	void (*rstn)(bool);
	void (*bootn)(bool);
	sh2_rxCallback_t *onRx;
	void *onRxCookie;
	uint16_t addr;
	uint8_t rxBuf[64]; // CHECK_ADAM
	uint16_t rxRemaining;
} Sh2Hal_t;

Sh2Hal_t g_sh2_Hal;


// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
// Callbacks
static void INTN_IRQ(void);
static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent);

// I/O
static void rstn0(bool state);
static void bootn0(bool state);
static int i2cBlockingRx(unsigned addr, uint8_t* pData, unsigned len);
static int i2cBlockingTx(unsigned addr, uint8_t* pData, unsigned len);


// ----------------------------------------------------------------------------------
// Callbacks and Handlers 
// ----------------------------------------------------------------------------------
static void INTN_IRQ()
{
	g_sh2_imu_event.t_ms = millis();
	g_sh2_imu_event.id = EVT_INTN;
}

static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
	if (pEvent->eventId == SH2_RESET) {
		g_sh2_resetPerformed = true;
	}
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent)
{
	g_sh2_sensorEvent = *pEvent;
	g_sh2_sensorReceived = true;
}

static void bootn0(bool state)
{
	digitalWrite(g_sh2_sensorPins.pin_bootn, state ? HIGH : LOW);
}

static void rstn0(bool state)
{
	digitalWrite(g_sh2_sensorPins.pin_rstn, state ? HIGH : LOW);
}

int i2cBlockingTx(unsigned addr, uint8_t* pData, unsigned len)
{
	addr = addr >> 1;
	
	Wire.beginTransmission(addr);
		
	for(uint8_t i = 0; i < len; i++){
		uint8_t databyte = (uint8_t)(pData[i]);
		volatile uint8_t success = Wire.write(databyte);
	}
	
	uint8_t result = Wire.endTransmission();
	
	if(result == 0)
	{
		return SH2_OK;
	}
	else
	{
		return SH2_ERR_IO;
	}
}

int i2cBlockingRx(unsigned addr, uint8_t *pData, unsigned len)
{
	addr = addr >> 1;
	
	Wire.requestFrom(addr, len);
	
	uint8_t ack = false;
	
	uint8_t i = 0;
				
	uint8_t *readPtr = pData;		

	/// @todo: Might write over memory if pData too small...				
	while(Wire.available())
	{
		*readPtr = Wire.read();
		readPtr++;
		ack = true;
	}
				
	if(ack == true)
	{
		return SH2_OK;
	}
	else
	{
		return SH2_ERR_IO;
	}
}

int sh2_hal_tx(uint8_t *pData, uint32_t len)
{
	// Do nothing if len is zero
	if (len == 0) {
		return SH2_OK;
	}

	// Do tx, and return when done
	return i2cBlockingTx(g_sh2_Hal.addr, pData, len);
}	

int sh2_hal_rx(uint8_t *pData, uint32_t len)
{
	// Do nothing if len is zero
	if (len == 0) {
		return SH2_OK;
	}

	// do rx and return when done
	return i2cBlockingRx(g_sh2_Hal.addr, pData, len);
}

int sh2_hal_block(void){
	return SH2_OK;
};

int sh2_hal_unblock(void){
	return SH2_OK;
}

int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie)
{
	// Store params for later reference
	g_sh2_Hal.onRxCookie = cookie;
	g_sh2_Hal.onRx = onRx;

	// Set addr to use in this mode
	g_sh2_Hal.addr = dfuMode ? SH2_i2c_addr_dfu_0<<1 : SH2_i2c_addr_0<<1;
	
	// Assert reset
	g_sh2_Hal.rstn(false);
	
	// Set BOOTN according to dfuMode
	g_sh2_Hal.bootn(dfuMode ? 0 : 1);

	// Wait for reset to take effect
	delay(SH2_reset_delay);

	// Enable INTN interrupt
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	attachInterrupt(g_sh2_sensorPins.pin_intn, INTN_IRQ, FALLING);
	
	// Deassert reset
	g_sh2_Hal.rstn(true);
	
	// If reset into DFU mode, wait until bootloader should be ready
	if (dfuMode) {
		delay(SH2_dfu_delay);
	}

	// Will need to reset the i2c peripheral after this.
	g_sh2_sensorReceived = true;
	
	return SH2_OK;
}
	
void Hillcrest_ :: math_rotate(sh2_Euler_t euler, sh2_Quaternion_t * quaterinon)
{
	sh2_Euler_t radians;
		
	radians.x = euler.x * scaleDegToRad;
	radians.y = euler.y * scaleDegToRad;
	radians.z = euler.z * scaleDegToRad;
		
	// Assuming the angles are in radians.
	float c1 = cosf(radians.y);
	float s1 = sinf(radians.y);
		
	float c2 = cosf(radians.z);
	float s2 = sinf(radians.z);
		
	float c3 = cosf(radians.x);
	float s3 = sinf(radians.x);
		
	quaterinon -> w = sqrtf(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;

	float w4 = quaterinon -> w * 4.0;

	quaterinon -> x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
	quaterinon -> y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
	quaterinon -> z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
}
	

static void set_orientation_forced(void)
{
// 	config0[0] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.x);
// 	config0[1] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.y);
// 	config0[2] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.z);
// 	config0[3] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.w);
// 		
// 	int status_pre = sh2_setFrs(SYSTEM_ORIENTATION, &config0[1], ARRAY_LEN(config0));
}


// ----------------------------------------------------------------------------------
// Public
Hillcrest_::Hillcrest_(void)
{
};

int8_t Hillcrest_:: begin(sh2_SensorPins_t setupPins)
{
	g_sh2_sensorPins.pin_rstn	= setupPins.pin_rstn;
	g_sh2_sensorPins.pin_bootn	= setupPins.pin_bootn;
	g_sh2_sensorPins.pin_intn	= setupPins.pin_intn;
	
	sh2_hal_init();
	
	// init SHTP layer
	shtp_init();

	g_sh2_resetPerformed = false;
	g_sh2_startedReports = false;

	// init SH2 layer
	sh2_initialize(eventHandler, NULL);
	//sh2_reinitialize(void);
	
	// Register event listener
	sh2_setSensorCallback(sensorHandler, NULL);

	// wait for reset notification, or just go ahead after 100ms
	int waited = 0;
	
	while (!g_sh2_resetPerformed && (waited < 200))
	{
		delay(1);
		waited++;
	}
}

void Hillcrest_:: end(void)
{
	detachInterrupt(g_sh2_sensorPins.pin_intn);
	sh2_reinitialize();
	pinMode(g_sh2_sensorPins.pin_bootn, INPUT);	
	pinMode(g_sh2_sensorPins.pin_rstn, INPUT);
};

void Hillcrest_ :: configure(sh2_SensorType_t sensorType, sh2_SensorInterrupts_t sensorInts, sh2_SensorDMPConfig_t sensorDMP, sh2_Euler_t sensorOrientation)
{
	g_sh2_sensorType = sensorType;
	
	g_sh2_sensorReportConfig.reportInterval_us			= sensorInts.report_interval_microseconds;
	g_sh2_sensorReportConfig.alwaysOnEnabled			= sensorInts.enable_always_on;
	g_sh2_sensorReportConfig.changeSensitivityEnabled	= sensorInts.enable_reports_on_change;
	g_sh2_sensorReportConfig.changeSensitivityRelative	= sensorInts.enable_change_relativity;
	g_sh2_sensorReportConfig.changeSensitivity			= sensorInts.change_threshold;
	g_sh2_sensorReportConfig.wakeupEnabled				= sensorInts.enable_wakeup;
	
	g_sh2_settings_DMP = sensorDMP;
	
	//math_rotate(sensorOrientation, &g_sh2_settings_sensorOrientation);
	
	g_sh2_settings_sensorOrientation.w = 0;
	g_sh2_settings_sensorOrientation.x = 0;
	g_sh2_settings_sensorOrientation.y = 1;
	g_sh2_settings_sensorOrientation.z = 0;
	
	sh2_onReset();
}

void Hillcrest_ :: startReports(void)
{
	sh2_startReports();	
}

bool Hillcrest_ :: checkReport(sh2_SensorValue_t * values){
	sh2_halTask();
	
	uint8_t sh2_state_changed = false;
	
	if (g_sh2_sensorReceived)
	{
		g_sh2_sensorReceived = false;
		sh2_state_changed = true;
		sh2_parseEvent(&g_sh2_sensorEvent, values);
	}

	if (g_sh2_resetPerformed)
	{
		sh2_onReset();
	}
	
	return sh2_state_changed;
}

int8_t Hillcrest_ :: tare(bool tare_x, bool tare_y, bool tare_z, sh2_TareBasis_t basis, bool persist)
{
	uint8_t axes = (tare_x ? SH2_TARE_X : 0) | (tare_y ? SH2_TARE_Y : 0) | (tare_z ? SH2_TARE_Z : 0);
	int8_t result = sh2_setTareNow(axes, basis);

	if(persist == true)
	{
		sh2_persistTare();		
	}
	
	return result;
}

int8_t Hillcrest_ :: saveDCD(void)
{
	return sh2_saveDcdNow();
}

int8_t Hillcrest_ :: setDCD(bool accel, bool gyro, bool mag)
{
	g_sh2_sensorDCDConfig.enable_accel_dcd = accel;
	g_sh2_sensorDCDConfig.enable_gyro_dcd = gyro;
	g_sh2_sensorDCDConfig.enable_mag_dcd = mag;
	
	return sh2_setCalConfig(
		(g_sh2_sensorDCDConfig.enable_accel_dcd ? SH2_CAL_ACCEL : 0x0) |
		(g_sh2_sensorDCDConfig.enable_gyro_dcd ? SH2_CAL_GYRO : 0x0) |
		(g_sh2_sensorDCDConfig.enable_mag_dcd ? SH2_CAL_MAG : 0x0)
	);	
};

int8_t Hillcrest_ :: clearDCD(void)
{
	return sh2_clearDcdAndReset();
}

void Hillcrest_ :: debugProdIds(void)
{
	int status;
	
	sh2_ProductIds_t sh2_product_ids;

	memset(&sh2_product_ids, 0, sizeof(sh2_product_ids));
	
	status = sh2_getProdIds(&sh2_product_ids);
	
	//sh2_halTask();
	
	if (status < 0) {
		serial_out.printf("Error from sh2_getProdIds.\n");
		return;
	}

	// Report the results
	for (int n = 0; n < SH2_NUM_PROD_ID_ENTRIES; n++) {
		serial_out.printf("Part %d : Version %d.%d.%d Build %d\n",
		sh2_product_ids.entry[n].swPartNumber,
		sh2_product_ids.entry[n].swVersionMajor, sh2_product_ids.entry[n].swVersionMinor,
		sh2_product_ids.entry[n].swVersionPatch, sh2_product_ids.entry[n].swBuildNumber);
	}
}

void Hillcrest_ :: debugValues(sh2_SensorValue_t * values)
{
	float t = values -> timestamp / 1000000.0;
	
	switch (values -> sensorId) {
		case SH2_RAW_ACCELEROMETER:
		serial_out.print("Raw ACC | ");
		serial_out.print(values -> un.rawAccelerometer.x);
		serial_out.print("x | ");
		serial_out.print(values -> un.rawAccelerometer.y);
		serial_out.print("y | ");
		serial_out.print(values -> un.rawAccelerometer.z);
		serial_out.println("z");
		break;

		case SH2_ACCELEROMETER:
		serial_out.print("ACC | ");
		serial_out.print(values -> un.accelerometer.x, 3);
		serial_out.print("x | ");
		serial_out.print(values -> un.accelerometer.y, 3);
		serial_out.print("y | ");
		serial_out.print(values -> un.accelerometer.z, 3);
		serial_out.println("z");
		break;

		case SH2_ROTATION_VECTOR:
		serial_out.print("RV | ");
		serial_out.print(values -> un.rotationVector.real, 3);
		serial_out.print("r | ");
		serial_out.print(values -> un.rotationVector.i, 3);
		serial_out.print("i | ");
		serial_out.print(values -> un.rotationVector.j, 3);
		serial_out.print("j | ");
		serial_out.print(values -> un.rotationVector.k, 3);
		serial_out.print("k | accuracy: ");
		serial_out.print(values -> un.rotationVector.accuracy, 3);
		serial_out.print("rad | t: ");
		serial_out.print(t);
		serial_out.println("s");
		break;
		
		case SH2_GYRO_INTEGRATED_RV:
		serial_out.print("GIRV | ");
		serial_out.print(values -> un.gyroIntegratedRV.real, 3);
		serial_out.print("r | ");
		serial_out.print(values -> un.gyroIntegratedRV.i, 3);
		serial_out.print("i | ");
		serial_out.print(values -> un.gyroIntegratedRV.j, 3);
		serial_out.print("j | ");
		serial_out.print(values -> un.gyroIntegratedRV.k, 3);
		serial_out.print("k | ");
		serial_out.print(values -> un.gyroIntegratedRV.angVelX, 3);
		serial_out.print("x | ");
		serial_out.print(values -> un.gyroIntegratedRV.angVelY, 3);
		serial_out.print("y | ");
		serial_out.print(values -> un.gyroIntegratedRV.angVelZ, 3);
		serial_out.print("z | t: ");
		serial_out.print(t, 4);
		serial_out.println("s");
		break;
		
		default:
		serial_out.printf("Unknown sensor: %d\n", values -> sensorId);
		break;
	}
}

void Hillcrest_ :: integer_math_convert_euler(sh2_Euler_t euler, sh2_Quaternion_2Compliment_t * quaterinon, int8_t q_point)
{
	sh2_Euler_t radians;
	
	radians.x = euler.x * scaleDegToRad;
	radians.y = euler.y * scaleDegToRad;
	radians.z = euler.z * scaleDegToRad;
	
	// Assuming the angles are in radians.
	float c1 = cosf(radians.y);
	float s1 = sinf(radians.y);
	
	float c2 = cosf(radians.z);
	float s2 = sinf(radians.z);
	
	float c3 = cosf(radians.x);
	float s3 = sinf(radians.x);
	
	float real = sqrtf(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
	float r4 = real * 4.0;
	
	quaterinon -> real = twos_compliment(q_point, real);
	
	quaterinon -> i = twos_compliment(q_point, (float)((c2 * s3 + c1 * s3 + s1 * s2 * c3) / r4));
	quaterinon -> j = twos_compliment(q_point, (float)((s1 * c2 + s1 * c3 + c1 * s2 * s3) / r4));
	quaterinon -> k = twos_compliment(q_point, (float)((-s1 * s3 + c1 * s2 * c3 + s2) / r4));
	
	quaterinon -> q_point = q_point;
}

void Hillcrest_ :: integer_math_rotate_quaternion(sh2_Quaternion_2Compliment_t rotationVector, sh2_Quaternion_2Compliment_t * quaternion, int8_t q_point)
{
	int32_t quaternion_real =
	((quaternion -> real * rotationVector.real) >> q_point)	-
	((quaternion -> i * rotationVector.i) >> q_point) -
	((quaternion -> j * rotationVector.j) >> q_point) -
	((quaternion -> k * rotationVector.k) >> q_point);  // 1
	
	int32_t quaternion_i =
	((quaternion -> real * rotationVector.i) >> q_point) +
	((quaternion -> i * rotationVector.real) >> q_point) +
	((quaternion -> j * rotationVector.k) >> q_point) -
	((quaternion -> k * rotationVector.j) >> q_point);  // i
	
	int32_t quaternion_j =
	((quaternion -> real * rotationVector.j) >> q_point) -
	((quaternion -> i * rotationVector.k) >> q_point) +
	((quaternion -> j * rotationVector.real) >> q_point) +
	((quaternion -> k * rotationVector.i) >> q_point);  // j
	
	int32_t quaternion_k =
	((quaternion -> real * rotationVector.k) >> q_point) +
	((quaternion -> i * rotationVector.j) >> q_point) -
	((quaternion -> j * rotationVector.i) >> q_point) +
	((quaternion -> k * rotationVector.real) >> q_point);  // k
	
	quaternion -> real = quaternion_real;
	
	quaternion -> i = quaternion_i;
	quaternion -> j = quaternion_j;
	quaternion -> k = quaternion_k;
	
	quaternion -> q_point = q_point;
}


// ----------------------------------------------------------------------------------
// Private
void Hillcrest_ :: sh2_hal_init(void)
{
	Wire.begin();
	pinMode(g_sh2_sensorPins.pin_bootn, OUTPUT);
	pinMode(g_sh2_sensorPins.pin_rstn, OUTPUT);
		
	g_sh2_i2cResetNeeded = true;
		
	g_sh2_Hal.rstn = rstn0;
	g_sh2_Hal.bootn = bootn0;

	// Put SH2 device in reset
	g_sh2_Hal.rstn(false);  // Hold in reset
	g_sh2_Hal.bootn(true);  // SH-2, not DFU
};

void Hillcrest_ :: sh2_onReset(void)
{
	// Configure calibration config as we want it
	sh2_configure();

	// Start the flow of sensor reports
	sh2_startReports();
	
	// Toggle reset flag back to false
	g_sh2_resetPerformed = false;
}

void Hillcrest_ :: sh2_parseEvent(const sh2_SensorEvent_t * event, sh2_SensorValue_t * values)
{
	int retval;
	
	retval = sh2_decodeSensorEvent(values, event);
}

void Hillcrest_ :: sh2_configure(void)
{
	int status = SH2_OK;

	uint32_t config0[4];
	
	//SYSTEM_ORIENTATION
	if(true)
	{		
		config0[0] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.x);
		config0[1] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.y);
		config0[2] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.z);
		config0[3] = (uint32_t) twos_compliment(30, g_sh2_settings_sensorOrientation.w);
		
		int status_pre = sh2_setFrs(SYSTEM_ORIENTATION, &config0[1], ARRAY_LEN(config0));
		
		status = (status_pre != SH2_OK ? status_pre : status);
	}
		
	uint32_t GIRV_CONFIG[7];
	GIRV_CONFIG[1] = (uint32_t) g_sh2_settings_DMP.SH2_GIRV_SYNC_INTERVAL_US;
	GIRV_CONFIG[2] = (uint32_t) twos_compliment(29, g_sh2_settings_DMP.SH2_GIRV_MAX_ERR * scaleDegToRad);
	GIRV_CONFIG[3] = (uint32_t) twos_compliment(10, g_sh2_settings_DMP.GIRV_PRED_AMT_S);
	GIRV_CONFIG[4] = (uint32_t) twos_compliment(20, g_sh2_settings_DMP.SH2_GRIV_ALPHA);
	GIRV_CONFIG[5] = (uint32_t) twos_compliment(20, g_sh2_settings_DMP.SH2_GRIV_BETA);
	GIRV_CONFIG[6] = (uint32_t) twos_compliment(20, g_sh2_settings_DMP.SH2_GRIV_GAMMA);

	switch(g_sh2_sensorType)
	{
		case SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_WITH_MAG:
		case SH2_TYPE_AR_VR_STABILIZED_ROTATION_VECTOR:
			GIRV_CONFIG[0] = (uint32_t) 0x204;
			break;
		default:
			GIRV_CONFIG[0] = (uint32_t) 0x207;
			break;			
	}

	int status_pre = sh2_setFrs(GYRO_INTEGRATED_RV_CONFIG, &GIRV_CONFIG[0], ARRAY_LEN(GIRV_CONFIG));
		
	status = (status_pre != SH2_OK ? status_pre : status);		
	
	// Enable dynamic calibration for A, G and M sensors
	status = sh2_setCalConfig(
		(g_sh2_sensorDCDConfig.enable_accel_dcd ? SH2_CAL_ACCEL : 0x0) | 
		(g_sh2_sensorDCDConfig.enable_gyro_dcd ? SH2_CAL_GYRO : 0x0) | 
		(g_sh2_sensorDCDConfig.enable_mag_dcd ? SH2_CAL_MAG : 0x0)
		);
	
	if (status != SH2_OK) {
		// Error catch
	}
}

void Hillcrest_ :: sh2_startReports(void)
{
	int status;
	int sensorId;

	switch(g_sh2_sensorType)
	{
		case SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_NO_MAG:
		case SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_WITH_MAG:
			sensorId = SH2_GYRO_INTEGRATED_RV;
			break;
		case SH2_TYPE_ROTATION_VECTOR:
			sensorId = SH2_ROTATION_VECTOR;
			break;
		case SH2_TYPE_GAME_ROTATION_VECTOR:
			sensorId = SH2_GAME_ROTATION_VECTOR;
			break;
		case SH2_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
			sensorId = SH2_GEOMAGNETIC_ROTATION_VECTOR;
			break;
		case SH2_TYPE_AR_VR_STABILIZED_ROTATION_VECTOR:
			sensorId = SH2_ARVR_STABILIZED_RV;
			break;
		case SH2_TYPE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
			sensorId = SH2_ARVR_STABILIZED_GRV;
			break;
		default:
			SH2_TYPE_GYRO_INTEGRATED_ROTATION_VECTOR_NO_MAG;
			break;
	}
	
	sh2_setSensorConfig(sensorId, &g_sh2_sensorReportConfig);
}

void Hillcrest_ :: sh2_i2cReset(void)
{
	//serial_out.printf("I2C Bus Reset\n");
	g_sh2_i2cResetNeeded = false;
}

void Hillcrest_ :: sh2_halTask(void)
{
	//Event_t event;
	unsigned readLen = 0;
	unsigned cargoLen = 0;

	// Handle the event
	switch (g_sh2_imu_event.id) {
		case EVT_INTN:
			g_sh2_imu_event.id = EVT_NONE;
			
			// If no RX callback registered, don't bother trying to read
			if (g_sh2_Hal.onRx != 0) {
				// Compute read length
				readLen = g_sh2_Hal.rxRemaining;
				
				if (readLen < SH2_SHTP_header_length) {
					// always read at least the SHTP header
					readLen = SH2_SHTP_header_length;
				}
				
				if (readLen > SH2_HAL_MAX_TRANSFER) {
					// limit reads to transfer size
					readLen = SH2_HAL_MAX_TRANSFER;
				}

				// Read i2c
				i2cBlockingRx(g_sh2_Hal.addr, g_sh2_Hal.rxBuf, readLen);
				
				// Get total cargo length from SHTP header
				cargoLen = ((g_sh2_Hal.rxBuf[1] << 8) + (g_sh2_Hal.rxBuf[0])) & (~0x8000);
				
				// Re-Evaluate rxRemaining
				if (cargoLen > readLen) {
					// More to read.
					g_sh2_Hal.rxRemaining = (cargoLen - readLen) + SH2_SHTP_header_length;
				}
				else {
					// All done, next read should be header only.
					g_sh2_Hal.rxRemaining = 0;
				}

				// Deliver via onRx callback
				g_sh2_Hal.onRx(g_sh2_Hal.onRxCookie, g_sh2_Hal.rxBuf, readLen, g_sh2_imu_event.t_ms * 1000);
			}

			break;
		default:
			// Unknown event type.  Ignore.
			break;
	}
}

Hillcrest_ Hillcrest;
