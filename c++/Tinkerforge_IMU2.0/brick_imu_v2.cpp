/* ***********************************************************
 * This file was automatically generated on 2019-01-29.      *
 *                                                           *
 * C/C++ Bindings Version 2.1.24                             *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/


#define IPCON_EXPOSE_INTERNALS

#include "brick_imu_v2.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef void (*Acceleration_CallbackFunction)(int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*MagneticField_CallbackFunction)(int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*AngularVelocity_CallbackFunction)(int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*Temperature_CallbackFunction)(int8_t temperature, void *user_data);

typedef void (*LinearAcceleration_CallbackFunction)(int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*GravityVector_CallbackFunction)(int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*Orientation_CallbackFunction)(int16_t heading, int16_t roll, int16_t pitch, void *user_data);

typedef void (*Quaternion_CallbackFunction)(int16_t w, int16_t x, int16_t y, int16_t z, void *user_data);

typedef void (*AllData_CallbackFunction)(int16_t acceleration[3], int16_t magnetic_field[3], int16_t angular_velocity[3], int16_t euler_angle[3], int16_t quaternion[4], int16_t linear_acceleration[3], int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status, void *user_data);

#if defined _MSC_VER || defined __BORLANDC__
	#pragma pack(push)
	#pragma pack(1)
	#define ATTRIBUTE_PACKED
#elif defined __GNUC__
	#ifdef _WIN32
		// workaround struct packing bug in GCC 4.7 on Windows
		// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=52991
		#define ATTRIBUTE_PACKED __attribute__((gcc_struct, packed))
	#else
		#define ATTRIBUTE_PACKED __attribute__((packed))
	#endif
#else
	#error unknown compiler, do not know how to enable struct packing
#endif

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAcceleration_Request;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetAcceleration_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetMagneticField_Request;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetMagneticField_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAngularVelocity_Request;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetAngularVelocity_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetTemperature_Request;

typedef struct {
	PacketHeader header;
	int8_t temperature;
} ATTRIBUTE_PACKED GetTemperature_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetOrientation_Request;

typedef struct {
	PacketHeader header;
	int16_t heading;
	int16_t roll;
	int16_t pitch;
} ATTRIBUTE_PACKED GetOrientation_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetLinearAcceleration_Request;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetLinearAcceleration_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetGravityVector_Request;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetGravityVector_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetQuaternion_Request;

typedef struct {
	PacketHeader header;
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GetQuaternion_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAllData_Request;

typedef struct {
	PacketHeader header;
	int16_t acceleration[3];
	int16_t magnetic_field[3];
	int16_t angular_velocity[3];
	int16_t euler_angle[3];
	int16_t quaternion[4];
	int16_t linear_acceleration[3];
	int16_t gravity_vector[3];
	int8_t temperature;
	uint8_t calibration_status;
} ATTRIBUTE_PACKED GetAllData_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED LedsOn_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED LedsOff_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED AreLedsOn_Request;

typedef struct {
	PacketHeader header;
	uint8_t leds;
} ATTRIBUTE_PACKED AreLedsOn_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED SaveCalibration_Request;

typedef struct {
	PacketHeader header;
	uint8_t calibration_done;
} ATTRIBUTE_PACKED SaveCalibration_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetAccelerationPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAccelerationPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetAccelerationPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetMagneticFieldPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetMagneticFieldPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetMagneticFieldPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetAngularVelocityPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAngularVelocityPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetAngularVelocityPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetTemperaturePeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetTemperaturePeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetTemperaturePeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetOrientationPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetOrientationPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetOrientationPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetLinearAccelerationPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetLinearAccelerationPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetLinearAccelerationPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetGravityVectorPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetGravityVectorPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetGravityVectorPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetQuaternionPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetQuaternionPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetQuaternionPeriod_Response;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED SetAllDataPeriod_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetAllDataPeriod_Request;

typedef struct {
	PacketHeader header;
	uint32_t period;
} ATTRIBUTE_PACKED GetAllDataPeriod_Response;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED Acceleration_Callback;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED MagneticField_Callback;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED AngularVelocity_Callback;

typedef struct {
	PacketHeader header;
	int8_t temperature;
} ATTRIBUTE_PACKED Temperature_Callback;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED LinearAcceleration_Callback;

typedef struct {
	PacketHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED GravityVector_Callback;

typedef struct {
	PacketHeader header;
	int16_t heading;
	int16_t roll;
	int16_t pitch;
} ATTRIBUTE_PACKED Orientation_Callback;

typedef struct {
	PacketHeader header;
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
} ATTRIBUTE_PACKED Quaternion_Callback;

typedef struct {
	PacketHeader header;
	int16_t acceleration[3];
	int16_t magnetic_field[3];
	int16_t angular_velocity[3];
	int16_t euler_angle[3];
	int16_t quaternion[4];
	int16_t linear_acceleration[3];
	int16_t gravity_vector[3];
	int8_t temperature;
	uint8_t calibration_status;
} ATTRIBUTE_PACKED AllData_Callback;

typedef struct {
	PacketHeader header;
	uint8_t magnetometer_rate;
	uint8_t gyroscope_range;
	uint8_t gyroscope_bandwidth;
	uint8_t accelerometer_range;
	uint8_t accelerometer_bandwidth;
} ATTRIBUTE_PACKED SetSensorConfiguration_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetSensorConfiguration_Request;

typedef struct {
	PacketHeader header;
	uint8_t magnetometer_rate;
	uint8_t gyroscope_range;
	uint8_t gyroscope_bandwidth;
	uint8_t accelerometer_range;
	uint8_t accelerometer_bandwidth;
} ATTRIBUTE_PACKED GetSensorConfiguration_Response;

typedef struct {
	PacketHeader header;
	uint8_t mode;
} ATTRIBUTE_PACKED SetSensorFusionMode_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetSensorFusionMode_Request;

typedef struct {
	PacketHeader header;
	uint8_t mode;
} ATTRIBUTE_PACKED GetSensorFusionMode_Response;

typedef struct {
	PacketHeader header;
	uint8_t enable_dynamic_baudrate;
	uint32_t minimum_dynamic_baudrate;
} ATTRIBUTE_PACKED SetSPITFPBaudrateConfig_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetSPITFPBaudrateConfig_Request;

typedef struct {
	PacketHeader header;
	uint8_t enable_dynamic_baudrate;
	uint32_t minimum_dynamic_baudrate;
} ATTRIBUTE_PACKED GetSPITFPBaudrateConfig_Response;

typedef struct {
	PacketHeader header;
	uint8_t communication_method;
} ATTRIBUTE_PACKED GetSendTimeoutCount_Request;

typedef struct {
	PacketHeader header;
	uint32_t timeout_count;
} ATTRIBUTE_PACKED GetSendTimeoutCount_Response;

typedef struct {
	PacketHeader header;
	char bricklet_port;
	uint32_t baudrate;
} ATTRIBUTE_PACKED SetSPITFPBaudrate_Request;

typedef struct {
	PacketHeader header;
	char bricklet_port;
} ATTRIBUTE_PACKED GetSPITFPBaudrate_Request;

typedef struct {
	PacketHeader header;
	uint32_t baudrate;
} ATTRIBUTE_PACKED GetSPITFPBaudrate_Response;

typedef struct {
	PacketHeader header;
	char bricklet_port;
} ATTRIBUTE_PACKED GetSPITFPErrorCount_Request;

typedef struct {
	PacketHeader header;
	uint32_t error_count_ack_checksum;
	uint32_t error_count_message_checksum;
	uint32_t error_count_frame;
	uint32_t error_count_overflow;
} ATTRIBUTE_PACKED GetSPITFPErrorCount_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED EnableStatusLED_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED DisableStatusLED_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED IsStatusLEDEnabled_Request;

typedef struct {
	PacketHeader header;
	uint8_t enabled;
} ATTRIBUTE_PACKED IsStatusLEDEnabled_Response;

typedef struct {
	PacketHeader header;
	char port;
} ATTRIBUTE_PACKED GetProtocol1BrickletName_Request;

typedef struct {
	PacketHeader header;
	uint8_t protocol_version;
	uint8_t firmware_version[3];
	char name[40];
} ATTRIBUTE_PACKED GetProtocol1BrickletName_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetChipTemperature_Request;

typedef struct {
	PacketHeader header;
	int16_t temperature;
} ATTRIBUTE_PACKED GetChipTemperature_Response;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED Reset_Request;

typedef struct {
	PacketHeader header;
} ATTRIBUTE_PACKED GetIdentity_Request;

typedef struct {
	PacketHeader header;
	char uid[8];
	char connected_uid[8];
	char position;
	uint8_t hardware_version[3];
	uint8_t firmware_version[3];
	uint16_t device_identifier;
} ATTRIBUTE_PACKED GetIdentity_Response;

#if defined _MSC_VER || defined __BORLANDC__
	#pragma pack(pop)
#endif
#undef ATTRIBUTE_PACKED

static void imu_v2_callback_wrapper_acceleration(DevicePrivate *device_p, Packet *packet) {
	Acceleration_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ACCELERATION];
	Acceleration_Callback *callback = (Acceleration_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ACCELERATION];

	if (callback_function == NULL) {
		return;
	}

	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_magnetic_field(DevicePrivate *device_p, Packet *packet) {
	MagneticField_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_MAGNETIC_FIELD];
	MagneticField_Callback *callback = (MagneticField_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_MAGNETIC_FIELD];

	if (callback_function == NULL) {
		return;
	}

	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_angular_velocity(DevicePrivate *device_p, Packet *packet) {
	AngularVelocity_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ANGULAR_VELOCITY];
	AngularVelocity_Callback *callback = (AngularVelocity_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ANGULAR_VELOCITY];

	if (callback_function == NULL) {
		return;
	}

	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_temperature(DevicePrivate *device_p, Packet *packet) {
	Temperature_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_TEMPERATURE];
	Temperature_Callback *callback = (Temperature_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_TEMPERATURE];

	if (callback_function == NULL) {
		return;
	}

	callback_function(callback->temperature, user_data);
}

static void imu_v2_callback_wrapper_linear_acceleration(DevicePrivate *device_p, Packet *packet) {
	LinearAcceleration_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_LINEAR_ACCELERATION];
	LinearAcceleration_Callback *callback = (LinearAcceleration_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_LINEAR_ACCELERATION];

	if (callback_function == NULL) {
		return;
	}

	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_gravity_vector(DevicePrivate *device_p, Packet *packet) {
	GravityVector_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_GRAVITY_VECTOR];
	GravityVector_Callback *callback = (GravityVector_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_GRAVITY_VECTOR];

	if (callback_function == NULL) {
		return;
	}

	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_orientation(DevicePrivate *device_p, Packet *packet) {
	Orientation_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ORIENTATION];
	Orientation_Callback *callback = (Orientation_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ORIENTATION];

	if (callback_function == NULL) {
		return;
	}

	callback->heading = leconvert_int16_from(callback->heading);
	callback->roll = leconvert_int16_from(callback->roll);
	callback->pitch = leconvert_int16_from(callback->pitch);

	callback_function(callback->heading, callback->roll, callback->pitch, user_data);
}

static void imu_v2_callback_wrapper_quaternion(DevicePrivate *device_p, Packet *packet) {
	Quaternion_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_QUATERNION];
	Quaternion_Callback *callback = (Quaternion_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_QUATERNION];

	if (callback_function == NULL) {
		return;
	}

	callback->w = leconvert_int16_from(callback->w);
	callback->x = leconvert_int16_from(callback->x);
	callback->y = leconvert_int16_from(callback->y);
	callback->z = leconvert_int16_from(callback->z);

	callback_function(callback->w, callback->x, callback->y, callback->z, user_data);
}

static void imu_v2_callback_wrapper_all_data(DevicePrivate *device_p, Packet *packet) {
	AllData_CallbackFunction callback_function;
	void *user_data = device_p->registered_callback_user_data[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ALL_DATA];
	int i;
	AllData_Callback *callback = (AllData_Callback *)packet;

	*(void **)(&callback_function) = device_p->registered_callbacks[DEVICE_NUM_FUNCTION_IDS + IMU_V2_CALLBACK_ALL_DATA];

	if (callback_function == NULL) {
		return;
	}

	for (i = 0; i < 3; i++) callback->acceleration[i] = leconvert_int16_from(callback->acceleration[i]);
	for (i = 0; i < 3; i++) callback->magnetic_field[i] = leconvert_int16_from(callback->magnetic_field[i]);
	for (i = 0; i < 3; i++) callback->angular_velocity[i] = leconvert_int16_from(callback->angular_velocity[i]);
	for (i = 0; i < 3; i++) callback->euler_angle[i] = leconvert_int16_from(callback->euler_angle[i]);
	for (i = 0; i < 4; i++) callback->quaternion[i] = leconvert_int16_from(callback->quaternion[i]);
	for (i = 0; i < 3; i++) callback->linear_acceleration[i] = leconvert_int16_from(callback->linear_acceleration[i]);
	for (i = 0; i < 3; i++) callback->gravity_vector[i] = leconvert_int16_from(callback->gravity_vector[i]);

	callback_function(callback->acceleration, callback->magnetic_field, callback->angular_velocity, callback->euler_angle, callback->quaternion, callback->linear_acceleration, callback->gravity_vector, callback->temperature, callback->calibration_status, user_data);
}

void imu_v2_create(IMUV2 *imu_v2, const char *uid, IPConnection *ipcon) {
	DevicePrivate *device_p;

	device_create(imu_v2, uid, ipcon->p, 2, 0, 3);

	device_p = imu_v2->p;

	device_p->response_expected[IMU_V2_FUNCTION_GET_ACCELERATION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_MAGNETIC_FIELD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ANGULAR_VELOCITY] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_TEMPERATURE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ORIENTATION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_LINEAR_ACCELERATION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_GRAVITY_VECTOR] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_QUATERNION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ALL_DATA] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_LEDS_ON] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_LEDS_OFF] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_ARE_LEDS_ON] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SAVE_CALIBRATION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_ACCELERATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ACCELERATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_MAGNETIC_FIELD_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_MAGNETIC_FIELD_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_ANGULAR_VELOCITY_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ANGULAR_VELOCITY_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_TEMPERATURE_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_TEMPERATURE_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_ORIENTATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ORIENTATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_LINEAR_ACCELERATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_LINEAR_ACCELERATION_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_GRAVITY_VECTOR_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_GRAVITY_VECTOR_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_QUATERNION_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_QUATERNION_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_ALL_DATA_PERIOD] = DEVICE_RESPONSE_EXPECTED_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_ALL_DATA_PERIOD] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_SENSOR_CONFIGURATION] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SENSOR_CONFIGURATION] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_SENSOR_FUSION_MODE] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SENSOR_FUSION_MODE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_SPITFP_BAUDRATE_CONFIG] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SPITFP_BAUDRATE_CONFIG] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SEND_TIMEOUT_COUNT] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_SET_SPITFP_BAUDRATE] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SPITFP_BAUDRATE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_SPITFP_ERROR_COUNT] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_ENABLE_STATUS_LED] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_DISABLE_STATUS_LED] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_IS_STATUS_LED_ENABLED] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_PROTOCOL1_BRICKLET_NAME] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_CHIP_TEMPERATURE] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;
	device_p->response_expected[IMU_V2_FUNCTION_RESET] = DEVICE_RESPONSE_EXPECTED_FALSE;
	device_p->response_expected[IMU_V2_FUNCTION_GET_IDENTITY] = DEVICE_RESPONSE_EXPECTED_ALWAYS_TRUE;

	device_p->callback_wrappers[IMU_V2_CALLBACK_ACCELERATION] = imu_v2_callback_wrapper_acceleration;
	device_p->callback_wrappers[IMU_V2_CALLBACK_MAGNETIC_FIELD] = imu_v2_callback_wrapper_magnetic_field;
	device_p->callback_wrappers[IMU_V2_CALLBACK_ANGULAR_VELOCITY] = imu_v2_callback_wrapper_angular_velocity;
	device_p->callback_wrappers[IMU_V2_CALLBACK_TEMPERATURE] = imu_v2_callback_wrapper_temperature;
	device_p->callback_wrappers[IMU_V2_CALLBACK_LINEAR_ACCELERATION] = imu_v2_callback_wrapper_linear_acceleration;
	device_p->callback_wrappers[IMU_V2_CALLBACK_GRAVITY_VECTOR] = imu_v2_callback_wrapper_gravity_vector;
	device_p->callback_wrappers[IMU_V2_CALLBACK_ORIENTATION] = imu_v2_callback_wrapper_orientation;
	device_p->callback_wrappers[IMU_V2_CALLBACK_QUATERNION] = imu_v2_callback_wrapper_quaternion;
	device_p->callback_wrappers[IMU_V2_CALLBACK_ALL_DATA] = imu_v2_callback_wrapper_all_data;

}

void imu_v2_destroy(IMUV2 *imu_v2) {
	device_release(imu_v2->p);
}

int imu_v2_get_response_expected(IMUV2 *imu_v2, uint8_t function_id, bool *ret_response_expected) {
	return device_get_response_expected(imu_v2->p, function_id, ret_response_expected);
}

int imu_v2_set_response_expected(IMUV2 *imu_v2, uint8_t function_id, bool response_expected) {
	return device_set_response_expected(imu_v2->p, function_id, response_expected);
}

int imu_v2_set_response_expected_all(IMUV2 *imu_v2, bool response_expected) {
	return device_set_response_expected_all(imu_v2->p, response_expected);
}

void imu_v2_register_callback(IMUV2 *imu_v2, int16_t callback_id, void *function, void *user_data) {
	device_register_callback(imu_v2->p, callback_id, function, user_data);
}

int imu_v2_get_api_version(IMUV2 *imu_v2, uint8_t ret_api_version[3]) {
	return device_get_api_version(imu_v2->p, ret_api_version);
}

int imu_v2_get_acceleration(IMUV2 *imu_v2, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetAcceleration_Request request;
	GetAcceleration_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ACCELERATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_magnetic_field(IMUV2 *imu_v2, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetMagneticField_Request request;
	GetMagneticField_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_MAGNETIC_FIELD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_angular_velocity(IMUV2 *imu_v2, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetAngularVelocity_Request request;
	GetAngularVelocity_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ANGULAR_VELOCITY, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_temperature(IMUV2 *imu_v2, int8_t *ret_temperature) {
	DevicePrivate *device_p = imu_v2->p;
	GetTemperature_Request request;
	GetTemperature_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_TEMPERATURE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_temperature = response.temperature;

	return ret;
}

int imu_v2_get_orientation(IMUV2 *imu_v2, int16_t *ret_heading, int16_t *ret_roll, int16_t *ret_pitch) {
	DevicePrivate *device_p = imu_v2->p;
	GetOrientation_Request request;
	GetOrientation_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ORIENTATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_heading = leconvert_int16_from(response.heading);
	*ret_roll = leconvert_int16_from(response.roll);
	*ret_pitch = leconvert_int16_from(response.pitch);

	return ret;
}

int imu_v2_get_linear_acceleration(IMUV2 *imu_v2, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetLinearAcceleration_Request request;
	GetLinearAcceleration_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_LINEAR_ACCELERATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_gravity_vector(IMUV2 *imu_v2, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetGravityVector_Request request;
	GetGravityVector_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_GRAVITY_VECTOR, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_quaternion(IMUV2 *imu_v2, int16_t *ret_w, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z) {
	DevicePrivate *device_p = imu_v2->p;
	GetQuaternion_Request request;
	GetQuaternion_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_QUATERNION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_w = leconvert_int16_from(response.w);
	*ret_x = leconvert_int16_from(response.x);
	*ret_y = leconvert_int16_from(response.y);
	*ret_z = leconvert_int16_from(response.z);

	return ret;
}

int imu_v2_get_all_data(IMUV2 *imu_v2, int16_t ret_acceleration[3], int16_t ret_magnetic_field[3], int16_t ret_angular_velocity[3], int16_t ret_euler_angle[3], int16_t ret_quaternion[4], int16_t ret_linear_acceleration[3], int16_t ret_gravity_vector[3], int8_t *ret_temperature, uint8_t *ret_calibration_status) {
	DevicePrivate *device_p = imu_v2->p;
	GetAllData_Request request;
	GetAllData_Response response;
	int ret;
	int i;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ALL_DATA, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	for (i = 0; i < 3; i++) ret_acceleration[i] = leconvert_int16_from(response.acceleration[i]);
	for (i = 0; i < 3; i++) ret_magnetic_field[i] = leconvert_int16_from(response.magnetic_field[i]);
	for (i = 0; i < 3; i++) ret_angular_velocity[i] = leconvert_int16_from(response.angular_velocity[i]);
	for (i = 0; i < 3; i++) ret_euler_angle[i] = leconvert_int16_from(response.euler_angle[i]);
	for (i = 0; i < 4; i++) ret_quaternion[i] = leconvert_int16_from(response.quaternion[i]);
	for (i = 0; i < 3; i++) ret_linear_acceleration[i] = leconvert_int16_from(response.linear_acceleration[i]);
	for (i = 0; i < 3; i++) ret_gravity_vector[i] = leconvert_int16_from(response.gravity_vector[i]);
	*ret_temperature = response.temperature;
	*ret_calibration_status = response.calibration_status;

	return ret;
}

int imu_v2_leds_on(IMUV2 *imu_v2) {
	DevicePrivate *device_p = imu_v2->p;
	LedsOn_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_LEDS_ON, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_leds_off(IMUV2 *imu_v2) {
	DevicePrivate *device_p = imu_v2->p;
	LedsOff_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_LEDS_OFF, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_are_leds_on(IMUV2 *imu_v2, bool *ret_leds) {
	DevicePrivate *device_p = imu_v2->p;
	AreLedsOn_Request request;
	AreLedsOn_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_ARE_LEDS_ON, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_leds = response.leds != 0;

	return ret;
}

int imu_v2_save_calibration(IMUV2 *imu_v2, bool *ret_calibration_done) {
	DevicePrivate *device_p = imu_v2->p;
	SaveCalibration_Request request;
	SaveCalibration_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SAVE_CALIBRATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_calibration_done = response.calibration_done != 0;

	return ret;
}

int imu_v2_set_acceleration_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetAccelerationPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_ACCELERATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_acceleration_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetAccelerationPeriod_Request request;
	GetAccelerationPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ACCELERATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_magnetic_field_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetMagneticFieldPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_MAGNETIC_FIELD_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_magnetic_field_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetMagneticFieldPeriod_Request request;
	GetMagneticFieldPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_MAGNETIC_FIELD_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_angular_velocity_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetAngularVelocityPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_ANGULAR_VELOCITY_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_angular_velocity_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetAngularVelocityPeriod_Request request;
	GetAngularVelocityPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ANGULAR_VELOCITY_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_temperature_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetTemperaturePeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_TEMPERATURE_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_temperature_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetTemperaturePeriod_Request request;
	GetTemperaturePeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_TEMPERATURE_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_orientation_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetOrientationPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_ORIENTATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_orientation_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetOrientationPeriod_Request request;
	GetOrientationPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ORIENTATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_linear_acceleration_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetLinearAccelerationPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_LINEAR_ACCELERATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_linear_acceleration_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetLinearAccelerationPeriod_Request request;
	GetLinearAccelerationPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_LINEAR_ACCELERATION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_gravity_vector_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetGravityVectorPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_GRAVITY_VECTOR_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_gravity_vector_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetGravityVectorPeriod_Request request;
	GetGravityVectorPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_GRAVITY_VECTOR_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_quaternion_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetQuaternionPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_QUATERNION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_quaternion_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetQuaternionPeriod_Request request;
	GetQuaternionPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_QUATERNION_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_all_data_period(IMUV2 *imu_v2, uint32_t period) {
	DevicePrivate *device_p = imu_v2->p;
	SetAllDataPeriod_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_ALL_DATA_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.period = leconvert_uint32_to(period);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_all_data_period(IMUV2 *imu_v2, uint32_t *ret_period) {
	DevicePrivate *device_p = imu_v2->p;
	GetAllDataPeriod_Request request;
	GetAllDataPeriod_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_ALL_DATA_PERIOD, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_period = leconvert_uint32_from(response.period);

	return ret;
}

int imu_v2_set_sensor_configuration(IMUV2 *imu_v2, uint8_t magnetometer_rate, uint8_t gyroscope_range, uint8_t gyroscope_bandwidth, uint8_t accelerometer_range, uint8_t accelerometer_bandwidth) {
	DevicePrivate *device_p = imu_v2->p;
	SetSensorConfiguration_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_SENSOR_CONFIGURATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.magnetometer_rate = magnetometer_rate;
	request.gyroscope_range = gyroscope_range;
	request.gyroscope_bandwidth = gyroscope_bandwidth;
	request.accelerometer_range = accelerometer_range;
	request.accelerometer_bandwidth = accelerometer_bandwidth;

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_sensor_configuration(IMUV2 *imu_v2, uint8_t *ret_magnetometer_rate, uint8_t *ret_gyroscope_range, uint8_t *ret_gyroscope_bandwidth, uint8_t *ret_accelerometer_range, uint8_t *ret_accelerometer_bandwidth) {
	DevicePrivate *device_p = imu_v2->p;
	GetSensorConfiguration_Request request;
	GetSensorConfiguration_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SENSOR_CONFIGURATION, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_magnetometer_rate = response.magnetometer_rate;
	*ret_gyroscope_range = response.gyroscope_range;
	*ret_gyroscope_bandwidth = response.gyroscope_bandwidth;
	*ret_accelerometer_range = response.accelerometer_range;
	*ret_accelerometer_bandwidth = response.accelerometer_bandwidth;

	return ret;
}

int imu_v2_set_sensor_fusion_mode(IMUV2 *imu_v2, uint8_t mode) {
	DevicePrivate *device_p = imu_v2->p;
	SetSensorFusionMode_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_SENSOR_FUSION_MODE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.mode = mode;

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_sensor_fusion_mode(IMUV2 *imu_v2, uint8_t *ret_mode) {
	DevicePrivate *device_p = imu_v2->p;
	GetSensorFusionMode_Request request;
	GetSensorFusionMode_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SENSOR_FUSION_MODE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_mode = response.mode;

	return ret;
}

int imu_v2_set_spitfp_baudrate_config(IMUV2 *imu_v2, bool enable_dynamic_baudrate, uint32_t minimum_dynamic_baudrate) {
	DevicePrivate *device_p = imu_v2->p;
	SetSPITFPBaudrateConfig_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_SPITFP_BAUDRATE_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.enable_dynamic_baudrate = enable_dynamic_baudrate ? 1 : 0;
	request.minimum_dynamic_baudrate = leconvert_uint32_to(minimum_dynamic_baudrate);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_spitfp_baudrate_config(IMUV2 *imu_v2, bool *ret_enable_dynamic_baudrate, uint32_t *ret_minimum_dynamic_baudrate) {
	DevicePrivate *device_p = imu_v2->p;
	GetSPITFPBaudrateConfig_Request request;
	GetSPITFPBaudrateConfig_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SPITFP_BAUDRATE_CONFIG, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_enable_dynamic_baudrate = response.enable_dynamic_baudrate != 0;
	*ret_minimum_dynamic_baudrate = leconvert_uint32_from(response.minimum_dynamic_baudrate);

	return ret;
}

int imu_v2_get_send_timeout_count(IMUV2 *imu_v2, uint8_t communication_method, uint32_t *ret_timeout_count) {
	DevicePrivate *device_p = imu_v2->p;
	GetSendTimeoutCount_Request request;
	GetSendTimeoutCount_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SEND_TIMEOUT_COUNT, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.communication_method = communication_method;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_timeout_count = leconvert_uint32_from(response.timeout_count);

	return ret;
}

int imu_v2_set_spitfp_baudrate(IMUV2 *imu_v2, char bricklet_port, uint32_t baudrate) {
	DevicePrivate *device_p = imu_v2->p;
	SetSPITFPBaudrate_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_SET_SPITFP_BAUDRATE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.bricklet_port = bricklet_port;
	request.baudrate = leconvert_uint32_to(baudrate);

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_spitfp_baudrate(IMUV2 *imu_v2, char bricklet_port, uint32_t *ret_baudrate) {
	DevicePrivate *device_p = imu_v2->p;
	GetSPITFPBaudrate_Request request;
	GetSPITFPBaudrate_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SPITFP_BAUDRATE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.bricklet_port = bricklet_port;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_baudrate = leconvert_uint32_from(response.baudrate);

	return ret;
}

int imu_v2_get_spitfp_error_count(IMUV2 *imu_v2, char bricklet_port, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow) {
	DevicePrivate *device_p = imu_v2->p;
	GetSPITFPErrorCount_Request request;
	GetSPITFPErrorCount_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_SPITFP_ERROR_COUNT, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.bricklet_port = bricklet_port;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_error_count_ack_checksum = leconvert_uint32_from(response.error_count_ack_checksum);
	*ret_error_count_message_checksum = leconvert_uint32_from(response.error_count_message_checksum);
	*ret_error_count_frame = leconvert_uint32_from(response.error_count_frame);
	*ret_error_count_overflow = leconvert_uint32_from(response.error_count_overflow);

	return ret;
}

int imu_v2_enable_status_led(IMUV2 *imu_v2) {
	DevicePrivate *device_p = imu_v2->p;
	EnableStatusLED_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_ENABLE_STATUS_LED, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_disable_status_led(IMUV2 *imu_v2) {
	DevicePrivate *device_p = imu_v2->p;
	DisableStatusLED_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_DISABLE_STATUS_LED, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_is_status_led_enabled(IMUV2 *imu_v2, bool *ret_enabled) {
	DevicePrivate *device_p = imu_v2->p;
	IsStatusLEDEnabled_Request request;
	IsStatusLEDEnabled_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_IS_STATUS_LED_ENABLED, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_enabled = response.enabled != 0;

	return ret;
}

int imu_v2_get_protocol1_bricklet_name(IMUV2 *imu_v2, char port, uint8_t *ret_protocol_version, uint8_t ret_firmware_version[3], char ret_name[40]) {
	DevicePrivate *device_p = imu_v2->p;
	GetProtocol1BrickletName_Request request;
	GetProtocol1BrickletName_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_PROTOCOL1_BRICKLET_NAME, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	request.port = port;

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_protocol_version = response.protocol_version;
	memcpy(ret_firmware_version, response.firmware_version, 3 * sizeof(uint8_t));
	memcpy(ret_name, response.name, 40);

	return ret;
}

int imu_v2_get_chip_temperature(IMUV2 *imu_v2, int16_t *ret_temperature) {
	DevicePrivate *device_p = imu_v2->p;
	GetChipTemperature_Request request;
	GetChipTemperature_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_CHIP_TEMPERATURE, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	*ret_temperature = leconvert_int16_from(response.temperature);

	return ret;
}

int imu_v2_reset(IMUV2 *imu_v2) {
	DevicePrivate *device_p = imu_v2->p;
	Reset_Request request;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_RESET, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, NULL);

	return ret;
}

int imu_v2_get_identity(IMUV2 *imu_v2, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier) {
	DevicePrivate *device_p = imu_v2->p;
	GetIdentity_Request request;
	GetIdentity_Response response;
	int ret;

	ret = packet_header_create(&request.header, sizeof(request), IMU_V2_FUNCTION_GET_IDENTITY, device_p->ipcon_p, device_p);

	if (ret < 0) {
		return ret;
	}

	ret = device_send_request(device_p, (Packet *)&request, (Packet *)&response);

	if (ret < 0) {
		return ret;
	}

	memcpy(ret_uid, response.uid, 8);
	memcpy(ret_connected_uid, response.connected_uid, 8);
	*ret_position = response.position;
	memcpy(ret_hardware_version, response.hardware_version, 3 * sizeof(uint8_t));
	memcpy(ret_firmware_version, response.firmware_version, 3 * sizeof(uint8_t));
	*ret_device_identifier = leconvert_uint16_from(response.device_identifier);

	return ret;
}

#ifdef __cplusplus
}
#endif
