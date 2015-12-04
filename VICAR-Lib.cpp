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

int CheckStatus()
{
	modbus_read_registers(mb, STATUS_WORD, 1, StatusWord);
	printf("status is %u\n", StatusWord[0]);
	if ((StatusWord[0] & 128) == 128)
		return 1;
	else
		return 0;
}

void SetSpeedMode() {
	ControlWord[0] = 128; //10000000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void SetPositionAndSpeedMode()
{
	ControlWord[0] = 224; //11100000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void SetPositionControlWord() {
	ControlWord[0] = 6;		//110   up front for X-Y axes
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void Set_R_PositionControlWord() {
	ControlWord[0] = 8;     //1000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void Set_R_ChangeSpeedControlWord() {
	ControlWord[0] = 32;     //100000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void ResetControlWord() {
	ControlWord[0] = 0;
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
};
void Set_R_and_ChangeOfSpeed_ControlWord()
{
	ControlWord[0] = 40; //101000
	modbus_write_registers(mb, CONTROL_WORD, 1, ControlWord);
}


// TO BE REMOVED - a couple of math functions
int sign(float value)
{
	if (value >= 0)
		return 1;
	else
		return -1;
}
float mean(vector<float>& q)
{
	float m = 0;
	for (int i = 0; i < q.size(); ++i)
	{
		m += q[i];
	}
	m /= q.size();
	//printf("mean is %.2f\n", m);
	return m;
}



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
		//SetPositionAndSpeedMode();
		SetMass(1);
		SetPositionXY(10, 10, CART_SPEED, CART_SPEED);
		return 0;
	};

	VICARLIB_API int Connect(string address = "137.204.56.92", int port = 1024, int mass = 1) {

		printf("Default mass is %i\n", mass);
		mb = modbus_new_tcp(address.c_str(), port);
		modbus_set_slave(mb, 1);
		int connection = modbus_connect(mb);
		printf("Modbus connected\n");

		Sleep(2000);
		//SetPositionAndSpeedMode();
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
		GetCurrent();		//update values of current
		//r_position = GetPositionR();	//update r position
		float abs_current;
		R_current = (float)current[2];
		printf("R current is %.2f\n", R_current);
		//printf("vel is %.2f\n", GetVelocityR());
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
	};

	VICARLIB_API int GetMass() {
		return cart_mass;
	};

	VICARLIB_API float* GetVelocity()
	{
		modbus_read_registers(mb, INSTANT_X_VELOCITY, 6, VelocityArray);
		velocity[0] = (float)(((unsigned short)VelocityArray[0] * 65536) + (unsigned short)VelocityArray[1]) / 1000;
		velocity[1] = (float)(((unsigned short)VelocityArray[2] * 65536) + (unsigned short)VelocityArray[3]) / 1000;
		velocity[2] = (float)(((unsigned short)VelocityArray[4] * 65536) + (unsigned short)VelocityArray[5]) / 1000;
		return velocity;
	}

	VICARLIB_API float GetVelocityX() {
		uint16_t XArray[2];
		float x_velocity;
		modbus_read_registers(mb, INSTANT_X_VELOCITY, 2, XArray);
		x_velocity = (float)(((unsigned short)XArray[0] * 65536) + (unsigned short)XArray[1]) / 1000;
		return x_velocity;
	};

	VICARLIB_API float GetVelocityY() {
		uint16_t YArray[2];
		float y_velocity;
		modbus_read_registers(mb, INSTANT_Y_VELOCITY, 2, YArray);
		y_velocity = (float)(((unsigned short)YArray[0] * 65536) + (unsigned short)YArray[1]) / 1000;
		return y_velocity;
	};

	VICARLIB_API float GetVelocityR() {
		uint16_t RArray[2];
		float r_velocity;
		modbus_read_registers(mb, INSTANT_R_VELOCITY, 2, RArray);
		r_velocity = (float)(((unsigned short)RArray[0] * 65536) + (unsigned short)RArray[1]) / 1000;
		return r_velocity;
	};



	VICARLIB_API float* GetPosition() {
		modbus_read_registers(mb, INSTANT_LINEAR_X_POSITION, 6, PosArray);
		position[0] = (float)(((unsigned short)PosArray[0] * 65536) + (unsigned short)PosArray[1]) / 1000;
		position[1] = (float)(((unsigned short)PosArray[2] * 65536) + (unsigned short)PosArray[3]) / 1000;
		position[2] = (float)(((unsigned short)PosArray[4] * 65536) + (unsigned short)PosArray[5]) / 1000;
		return position;
	};

	VICARLIB_API float GetPositionX() {
		uint16_t XArray[2];
		float x_position;
		modbus_read_registers(mb, INSTANT_LINEAR_X_POSITION, 2, XArray);
		x_position = (float)(((unsigned short)XArray[0] * 65536) + (unsigned short)XArray[1]) / 1000;
		return x_position;
	};

	VICARLIB_API float GetPositionY() {
		uint16_t YArray[2];
		float y_position;
		modbus_read_registers(mb, INSTANT_LINEAR_Y_POSITION, 2, YArray);
		y_position = (float)(((unsigned short)YArray[0] * 65536) + (unsigned short)YArray[1]) / 1000;
		return y_position;
	};

	VICARLIB_API float GetPositionR() {
		uint16_t RotArray[2];
		float r_position;
		modbus_read_registers(mb, INSTANT_LINEAR_R_POSITION, 2, RotArray);
		r_position = (float)(((unsigned short)RotArray[0] * 65536) + (unsigned short)RotArray[1]) / 1000;
		return r_position;
	};

	VICARLIB_API uint16_t* GetParametersX() {
		modbus_read_registers(mb, X_DRIVE_SPEED, 3, XSpeedParameters);
		return XSpeedParameters;
	};

	VICARLIB_API uint16_t* GetParametersY() {
		modbus_read_registers(mb, Y_DRIVE_SPEED, 3, YSpeedParameters);
		return YSpeedParameters;
	};

	VICARLIB_API uint16_t* GetParametersR() {
		modbus_read_registers(mb, R_DRIVE_SPEED, 3, RSpeedParameters);
		return RSpeedParameters;
	};

	VICARLIB_API void SetParametersX(uint16_t kp, uint16_t ki, uint16_t reference_filter) {

		XSpeedParameters[0] = kp;
		XSpeedParameters[1] = ki;
		XSpeedParameters[2] = reference_filter;
		modbus_write_registers(mb, X_DRIVE_SPEED, 3, XSpeedParameters);
	};

	VICARLIB_API void SetParametersY(uint16_t kp, uint16_t ki, uint16_t reference_filter) {

		YSpeedParameters[0] = kp;
		YSpeedParameters[1] = ki;
		YSpeedParameters[2] = reference_filter;
		modbus_write_registers(mb, Y_DRIVE_SPEED, 3, YSpeedParameters);
	};

	VICARLIB_API void SetParametersR(uint16_t kp, uint16_t ki, uint16_t reference_filter) {
		RSpeedParameters[0] = kp;
		RSpeedParameters[1] = ki;
		RSpeedParameters[2] = reference_filter;
		modbus_write_registers(mb, R_DRIVE_SPEED, 3, RSpeedParameters);
	};

	VICARLIB_API void SetPositionXY(int pos_x, int pos_y, int speed_x, int speed_y) {

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


	// WORKING NOW ONLY ON SPEED MODE 
	VICARLIB_API void SetPositionR(float pos_r)
	{
		int pr = pos_r * 1000;
		//printf("pr is %i\n", pr);
		ResetControlWord();
		SetPos_R_Array[0] = (pr & 0xFFFF0000) / 65536;
		SetPos_R_Array[1] = pr & 0x0000FFFF;
		SetPos_R_Array[2] = (CART_DEFAULT_ROTATION_SPEED & 0xFFFF0000) / 65536;
		SetPos_R_Array[3] = CART_DEFAULT_ROTATION_SPEED & 0x0000FFFF;
		modbus_write_registers(mb, SET_R_POSITION, 4, SetPos_R_Array);
		Set_R_PositionControlWord();

		//uint16_t ChangeOfSpeed[2];
		//change_of_speed *= 1000;
		////ResetControlWord();
		//ChangeOfSpeed[0] = (change_of_speed & 0xFFFF0000) / 65536;
		//ChangeOfSpeed[1] = change_of_speed & 0x0000FFFF;
		//modbus_write_registers(mb, SET_R_CHANGE_OF_SPEED, 2, ChangeOfSpeed);
		//Set_R_ChangeSpeedControlWord();

		//under test
		//Set_R_and_ChangeOfSpeed_ControlWord();
	};

	VICARLIB_API void SetPositionR_withSpeed(float pos_r, int change_of_speed)
	{
		int pr = pos_r * 1000;
		//printf("pr is %i\n", pr);
		ResetControlWord();
		SetPos_R_Array[0] = (pr & 0xFFFF0000) / 65536;
		SetPos_R_Array[1] = pr & 0x0000FFFF;
		SetPos_R_Array[2] = (CART_DEFAULT_ROTATION_SPEED & 0xFFFF0000) / 65536;
		SetPos_R_Array[3] = CART_DEFAULT_ROTATION_SPEED & 0x0000FFFF;
		modbus_write_registers(mb, SET_R_POSITION, 4, SetPos_R_Array);
		//Set_R_PositionControlWord();

		uint16_t ChangeOfSpeed[2];
		change_of_speed *= 1000;
		ChangeOfSpeed[0] = (change_of_speed & 0xFFFF0000) / 65536;
		ChangeOfSpeed[1] = change_of_speed & 0x0000FFFF;
		modbus_write_registers(mb, SET_R_CHANGE_OF_SPEED, 2, ChangeOfSpeed);
		Set_R_and_ChangeOfSpeed_ControlWord();
	};

	VICARLIB_API void SetChangeOfSpeedR(int change_of_speed) {
		uint16_t ChangeOfSpeed[2];
		change_of_speed *= 1000;
		ResetControlWord();
		ChangeOfSpeed[0] = (change_of_speed & 0xFFFF0000) / 65536;
		ChangeOfSpeed[1] = change_of_speed & 0x0000FFFF;
		modbus_write_registers(mb, SET_R_CHANGE_OF_SPEED, 2, ChangeOfSpeed);
		Set_R_ChangeSpeedControlWord();
	};

	VICARLIB_API void SetSpeedWithSignR(int speed)	//expressed in RPM
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

	VICARLIB_API short* GetCurrent() {
		modbus_read_registers(mb, INSTANT_X_CURRENT, 6, CurrentArray);
		current[0] = (short)(((unsigned short)CurrentArray[0] * 65536) + (unsigned short)CurrentArray[1]);
		current[1] = (short)(((unsigned short)CurrentArray[2] * 65536) + (unsigned short)CurrentArray[3]);
		current[2] = (short)(((unsigned short)CurrentArray[4] * 65536) + (unsigned short)CurrentArray[5]);
		return current;
	};
	VICARLIB_API short GetCurrentX() {
		return current[0];
	};
	VICARLIB_API short GetCurrentY() {
		return current[1];
	};
	VICARLIB_API short GetCurrentR() {
		return current[2];
	};

	VICARLIB_API short GetCurrentR_fromRegister()
	{
		uint16_t _currentArray[2];
		modbus_read_registers(mb, INSTANT_R_CURRENT, 2, _currentArray);
		short _current = (short)(((unsigned short)_currentArray[0] * 65536) + (unsigned short)_currentArray[1]);
		return _current;
	}

	VICARLIB_API bool IsMoving() {
		modbus_read_registers(mb, 100, 1, StatusWord);
		bool moving_status = CHECK_BIT(StatusWord[0], 4) || CHECK_BIT(StatusWord[0], 5) || CHECK_BIT(StatusWord[0], 6); //bit 4 and 5
		return moving_status;
	};
	VICARLIB_API bool IsReady() {
		modbus_read_registers(mb, 100, 1, StatusWord);
		bool ready_status = CHECK_BIT(StatusWord[0], 7);
		return ready_status;
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


	/*******************************/
	/*	TESTING FUNCTIONS		   */
	/*******************************/

	VICARLIB_API void RotationTestWithSign()
	{

		float abs_current;
		R_current = (float)GetCurrentR_fromRegister();
		printf("current is %.2f\n", R_current);
		abs_current = abs(R_current);
		if (abs_current > 200)
		{
			if (R_current > 0)
				SetSpeedWithSignR(DEFAULT_R_SPEED);
			else
				SetSpeedWithSignR(-DEFAULT_R_SPEED);
		}	
		else
			SetSpeedWithSignR(0);
	};



}

