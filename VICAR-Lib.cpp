// VICAR-Lib.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "VICAR-Lib.h"

#include "modbus.h"
#include "modbus-tcp.h"
#include "modbus-tcp-private.h"

#include <fstream>
#include <thread>
#include <vector>
#include <list>
#include <chrono>

using namespace std;

// check bit status
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

static int working_counter;
static bool has_moved;

static uint16_t PosArray[6];
static uint16_t SetPosArray[8];
static uint16_t SetPos_R_Array[4];
static uint16_t SetDeltaPositionArray[4];
static uint16_t CurrentArray[6];
static uint16_t VelocityArray[6];
static uint16_t ControlWord[1];
static uint16_t StatusWord[1];

static uint16_t XSpeedParameters[3];
static uint16_t YSpeedParameters[3];
static uint16_t RSpeedParameters[3];

static int cart_mass;
static short current[3];
static float torque;
static float position[3];
static float r_position;
static float velocity[3];

static float r_position_delta;
static float R_current;
static vector<float> r_filter_list;
static float r_start_position;

static modbus_t *mb;

fstream file;

/*********************************/
/*	PRIVATE CONTROL SETTINGS	 */
/*********************************/

// CONTROL WORD
void SetPositionAndSpeedMode()
{
	ControlWord[0] = 224; //11100000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};

void ResetControlWord() {
	ControlWord[0] = 0;
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};


void SetParametersX(uint16_t kp, uint16_t ki, uint16_t reference_filter) {

	XSpeedParameters[0] = kp;
	XSpeedParameters[1] = ki;
	XSpeedParameters[2] = reference_filter;
	modbus_write_registers(mb, X_DRIVE_SPEED, 3, XSpeedParameters);
};

// SET PARAMETERS

void SetParametersY(uint16_t kp, uint16_t ki, uint16_t reference_filter) {

	YSpeedParameters[0] = kp;
	YSpeedParameters[1] = ki;
	YSpeedParameters[2] = reference_filter;
	modbus_write_registers(mb, Y_DRIVE_SPEED, 3, YSpeedParameters);
};

void SetParametersR(uint16_t kp, uint16_t ki, uint16_t reference_filter) {
	RSpeedParameters[0] = kp;
	RSpeedParameters[1] = ki;
	RSpeedParameters[2] = reference_filter;
	modbus_write_registers(mb, R_DRIVE_SPEED, 3, RSpeedParameters);
};

// SET POSITION

void SetPositionXY(int pos_x, int pos_y, int speed_x, int speed_y) {

	// convert to micrometers
	pos_x *= 1000;
	pos_y *= 1000;

	//ResetControlWord();
	SetPosArray[0] = (pos_x & 0xFFFF0000) / 65536;
	SetPosArray[1] = pos_x & 0x0000FFFF;
	SetPosArray[2] = (speed_x & 0xFFFF0000) / 65536;
	SetPosArray[3] = speed_x & 0x0000FFFF;
	SetPosArray[4] = (pos_y & 0xFFFF0000) / 65536;
	SetPosArray[5] = pos_y & 0x0000FFFF;
	SetPosArray[6] = (speed_y & 0xFFFF0000) / 65536;
	SetPosArray[7] = speed_y & 0x0000FFFF;
	modbus_write_registers(mb, SET_LINEAR_X_POSITION, 8, SetPosArray);
	SetPositionAndSpeedMode();		// set X,Y and R
};

// CHECK STATUS WORD
bool IsMoving() {
	modbus_read_registers(mb, 100, 1, StatusWord);
	bool moving_status = CHECK_BIT(StatusWord[0], 4) || CHECK_BIT(StatusWord[0], 5) || CHECK_BIT(StatusWord[0], 6); //bit 4 and 5
	return moving_status;
};
bool IsReady() {
	modbus_read_registers(mb, 100, 1, StatusWord);
	bool ready_status = CHECK_BIT(StatusWord[0], 7);
	return ready_status;
};

float* GetVelocity()
{
	modbus_read_registers(mb, INSTANT_X_VELOCITY, 6, VelocityArray);
	velocity[0] = (float)(((unsigned short)VelocityArray[0] * 65536) + (unsigned short)VelocityArray[1]) / 1000;
	velocity[1] = (float)(((unsigned short)VelocityArray[2] * 65536) + (unsigned short)VelocityArray[3]) / 1000;
	velocity[2] = (float)(((unsigned short)VelocityArray[4] * 65536) + (unsigned short)VelocityArray[5]) / 1000;
	return velocity;
}

float GetVelocityX() {
	uint16_t XArray[2];
	float x_velocity;
	modbus_read_registers(mb, INSTANT_X_VELOCITY, 2, XArray);
	x_velocity = (float)(((unsigned short)XArray[0] * 65536) + (unsigned short)XArray[1]) / 1000;
	return x_velocity;
};

float GetVelocityY() {
	uint16_t YArray[2];
	float y_velocity;
	modbus_read_registers(mb, INSTANT_Y_VELOCITY, 2, YArray);
	y_velocity = (float)(((unsigned short)YArray[0] * 65536) + (unsigned short)YArray[1]) / 1000;
	return y_velocity;
};

float GetVelocityR() {
	uint16_t RArray[2];
	float r_velocity;
	modbus_read_registers(mb, INSTANT_R_VELOCITY, 2, RArray);
	r_velocity = (float)(((unsigned short)RArray[0] * 65536) + (unsigned short)RArray[1]) / 1000;
	return r_velocity;
};



float* GetPosition() {
	modbus_read_registers(mb, INSTANT_LINEAR_X_POSITION, 6, PosArray);
	position[0] = (float)(((unsigned short)PosArray[0] * 65536) + (unsigned short)PosArray[1]) / 1000;
	position[1] = (float)(((unsigned short)PosArray[2] * 65536) + (unsigned short)PosArray[3]) / 1000;
	position[2] = (float)(((unsigned short)PosArray[4] * 65536) + (unsigned short)PosArray[5]) / 1000;
	return position;
};

float GetPositionX() {
	uint16_t XArray[2];
	float x_position;
	modbus_read_registers(mb, INSTANT_LINEAR_X_POSITION, 2, XArray);
	x_position = (float)(((unsigned short)XArray[0] * 65536) + (unsigned short)XArray[1]) / 1000;
	return x_position;
};

float GetPositionY() {
	uint16_t YArray[2];
	float y_position;
	modbus_read_registers(mb, INSTANT_LINEAR_Y_POSITION, 2, YArray);
	y_position = (float)(((unsigned short)YArray[0] * 65536) + (unsigned short)YArray[1]) / 1000;
	return y_position;
};

float GetPositionR() {
	uint16_t RotArray[2];
	float r_position;
	modbus_read_registers(mb, INSTANT_LINEAR_R_POSITION, 2, RotArray);
	r_position = (float)(((unsigned short)RotArray[0] * 65536) + (unsigned short)RotArray[1]) / 1000;
	return r_position;
};

uint16_t* GetParametersX() {
	modbus_read_registers(mb, X_DRIVE_SPEED, 3, XSpeedParameters);
	return XSpeedParameters;
};

uint16_t* GetParametersY() {
	modbus_read_registers(mb, Y_DRIVE_SPEED, 3, YSpeedParameters);
	return YSpeedParameters;
};

uint16_t* GetParametersR() {
	modbus_read_registers(mb, R_DRIVE_SPEED, 3, RSpeedParameters);
	return RSpeedParameters;
};




void SetSpeedWithSignR(int speed)	//expressed in RPM
{
	//CheckStatus();
	if (speed < 0)		//check the sign
	{
		speed = 65536 - abs(speed);
	}
	uint16_t ChangeOfSpeed[1];
	ChangeOfSpeed[0] = speed;
	modbus_write_registers(mb, SET_R_CHANGE_OF_SPEED_WITH_SIGN, 1, ChangeOfSpeed);

};

short* GetCurrent() {
	modbus_read_registers(mb, INSTANT_X_CURRENT, 6, CurrentArray);
	current[0] = (short)(((unsigned short)CurrentArray[0] * 65536) + (unsigned short)CurrentArray[1]);
	current[1] = (short)(((unsigned short)CurrentArray[2] * 65536) + (unsigned short)CurrentArray[3]);
	current[2] = (short)(((unsigned short)CurrentArray[4] * 65536) + (unsigned short)CurrentArray[5]);
	return current;
};
 short GetCurrentX() {
	return current[0];
};
 short GetCurrentY() {
	return current[1];
};
 short GetCurrentR() {
	return current[2];
};


extern "C" {

	VICARLIB_API int Test() {

		return 77;		//good number!
	};

	// JUST FOR UNITY
	VICARLIB_API int ConnectForUnity()
	{
		mb = modbus_new_tcp("137.204.56.92", 1024);
		modbus_set_slave(mb, 1);
		modbus_connect(mb);

		printf("Modbus connected\n");

		Sleep(2000);
		SetMass(1);
		SetPositionXY(10, 10, CART_SPEED, CART_SPEED);
		return 0;
	};

	VICARLIB_API int Connect(const char* address = "137.204.56.92", int port = 1024, int mass = 1) {

		printf("Default mass is %i\n", mass);
		mb = modbus_new_tcp(address, port);
		modbus_set_slave(mb, 1);
		int connection = modbus_connect(mb);
		printf("Modbus connected\n");

		Sleep(2000);
		SetMass(mass);
		SetPositionXY(10, 10, CART_SPEED, CART_SPEED);

		return connection;
	};

	VICARLIB_API int Close() {

		modbus_close(mb);
		modbus_free(mb);

		return 0;
	};
	VICARLIB_API void SetMass(int value) {

		//range from 1 to 10
		if (value == 1) {
			SetParametersX(90, 3, 1000);
			SetParametersY(90, 3, 1000);
		}
		else if (value == 2)
		{
			SetParametersX(150, 20, 1000);
			SetParametersY(150, 20, 1000);
		}
		else {
			SetParametersX(90, 3, 1000);
			SetParametersY(90, 3, 1000);
		}
		cart_mass = value;
	};

	VICARLIB_API void Update()
	{

		//auto start = std::chrono::high_resolution_clock::now();

		GetCurrent();		//update values of current
		float abs_current;
		R_current = (float)current[2];
		//printf("R current is %.2f\n", R_current);
		abs_current = abs(R_current);

		// check cart mass
		if (cart_mass < 1)
			cart_mass = 1;

		if (abs_current > R_CURRENT_THRESHOLD)  //300
		{
			if (R_current > 0)
				SetSpeedWithSignR(DEFAULT_R_SPEED / cart_mass);
			else
				SetSpeedWithSignR(-DEFAULT_R_SPEED / cart_mass);
		}
		else
			SetSpeedWithSignR(0);

		//measure the time elapsed
		//auto end = std::chrono::high_resolution_clock::now();
		//auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		//printf("time elapsed %i\n", elapsed);
	};

	VICARLIB_API int GetMass() {
		return cart_mass;
	};

	




	VICARLIB_API float GetForceX() {
		if (abs(current[0]) > XY_CURRENT_THRESHOLD || abs(current[1]) > XY_CURRENT_THRESHOLD)
			return (current[0] + current[1]) / R_CURRENT_THRESHOLD;	// at least 300 - 200
		else
			return 0.0f;
	};
	VICARLIB_API float GetForceY() {
		if (abs(current[0]) > XY_CURRENT_THRESHOLD || abs(current[1]) > XY_CURRENT_THRESHOLD)
			return (current[0] - current[1]) / (XY_CURRENT_THRESHOLD - 100);	// at least 300 - 200
		else
			return 0.0f;
	};

	// torque = dL/dt with L = moment of inertia x angular velocity
	// moment of inertia = m * r^2  = 1  first approximations
	// torque proportial to R current but for currents < 200 needs to stop

	VICARLIB_API float GetTorque() {
		if (abs(current[2]) > R_CURRENT_THRESHOLD)
		{
			torque = current[2];		//set  torque equal to R current  (just a model)
			return torque;
		}
		else
		{
			//torque = 0.0f;
			torque = torque / 1.2;
			if (torque < 5 && torque > -5)
				torque = 0.0f;
			/*if (torque >= 5)
				torque -= 5;
			else if (torque <= 5)
				torque += 5;
			else
				torque = 0.0f;*/
			return torque;
		}	
	};

	VICARLIB_API float GetAbsoluteAngle() {
		return GetPositionR();	//update r position
	};


}

