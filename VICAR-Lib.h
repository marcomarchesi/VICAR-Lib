// define symbols
#ifdef VICARLIB_EXPORTS
#define VICARLIB_API __declspec(dllexport)
#else
#define VICARLIB_API __declspec(dllimport)
#endif

#include <string>

//constants

#define INTERVAL  0.01
#define CART_SPEED 600
#define CART_DEFAULT_ROTATION_SPEED 1
#define CHANGE_OF_SPEED_SAMPLING_TIME 0.25f

#define R_C_ALPHA 0.5



#define DEFAULT_CART_MASS 1
#define DEFAULT_R_SPEED 20
#define R_CURRENT_THRESHOLD 200
#define XY_CURRENT_THRESHOLD 300   //think 200 will be better

#define MAX_COLLISION_FACTOR 14
#define COLLISION_OFFSET 1


#define EMPTY_CART_SPEED_KP 50
#define EMPTY_CART_SPEED_KI 2
#define DEFAULT_SPEED_REFERENCE_FILTER 1000

// addresses

#define STATUS_WORD 100
#define CONTROL_WORD 101
#define INSTANT_LINEAR_X_POSITION 102
#define INSTANT_LINEAR_Y_POSITION 104
#define INSTANT_LINEAR_R_POSITION 106

#define INSTANT_X_CURRENT 108
#define INSTANT_Y_CURRENT 110
#define INSTANT_R_CURRENT 112

#define INSTANT_DELTA_X_POSITION 114
#define SET_LINEAR_X_POSITION 118
#define SET_R_POSITION 126


#define X_DRIVE_SPEED 138
#define Y_DRIVE_SPEED 144
#define R_DRIVE_SPEED 150

#define INSTANT_X_VELOCITY	154
#define INSTANT_Y_VELOCITY	156
#define INSTANT_R_VELOCITY	158

#define SET_R_CHANGE_OF_SPEED 160
#define SET_R_CHANGE_OF_SPEED_WITH_SIGN 162



using namespace std;

// internal functions not necessary to declare

// Set up front for movement
void SetPositionControlWord();
void Set_R_PositionControlWord();
void Set_R_ChangeSpeedControlWord();
void SetSpeedMode();
// Reset all the Control Word
void ResetControlWord();

extern "C" {

	// PUBLIC API
	//connect to default ip 137.204.56.92 - test
	VICARLIB_API int Connect(string address, int port, int mass);
	// initialize the system with a defined mass
	VICARLIB_API int ConnectForUnity();
	// close modbus connection
	VICARLIB_API int Close();
	// update the system without any thread
	VICARLIB_API void Update();
	// set cart mass
	VICARLIB_API void SetMass(int value);
	// calculate force to apply
	VICARLIB_API float GetForceX();
	VICARLIB_API float GetForceY();
	VICARLIB_API float GetAbsoluteAngle();
	VICARLIB_API float GetTorque();


	/*************************************************/

	//TODO THE FOLLOWING FUNCTIONS HAVE TO BE PRIVATE
	//just a test function
	VICARLIB_API int Test();

	// get X,Y positions
	VICARLIB_API float* GetPosition();
	VICARLIB_API float GetPositionX();
	VICARLIB_API float GetPositionY();
	VICARLIB_API float GetPositionR();
	// set X,Y positions
	VICARLIB_API void SetPositionXY(int pos_x,int pos_y,int speed_x,int speed_y);
	// set R position
	VICARLIB_API void SetPositionR(float pos_r);
	// set position and change speed
	VICARLIB_API void SetPositionR_withSpeed(float pos_r, int change_of_speed);
	// set R change of speed
	VICARLIB_API void SetChangeOfSpeedR(int change_of_speed);
	VICARLIB_API void SetSpeedWithSignR(int speed);

	// get velocity
	VICARLIB_API float* GetVelocity();
	VICARLIB_API float GetVelocityX();
	VICARLIB_API float GetVelocityY();
	VICARLIB_API float GetVelocityR();
	// get current from X,Y
	VICARLIB_API short* GetCurrent();
	VICARLIB_API short GetCurrentX();
	VICARLIB_API short GetCurrentY();
	VICARLIB_API short GetCurrentR();
	VICARLIB_API short GetCurrentR_fromRegister();

	// get drive parameters
	VICARLIB_API uint16_t* GetParametersX();
	VICARLIB_API uint16_t* GetParametersY();
	VICARLIB_API uint16_t* GetParametersR();

	// set drive parameters
	VICARLIB_API void SetParametersX(uint16_t kp, uint16_t ki, uint16_t reference_filter);
	VICARLIB_API void SetParametersY(uint16_t kp, uint16_t ki, uint16_t reference_filter);
	VICARLIB_API void SetParametersR(uint16_t kp, uint16_t ki, uint16_t reference_filter);

	// check if the system is moving
	VICARLIB_API bool IsMoving();
	// check if the system is ready
	VICARLIB_API bool IsReady();

	VICARLIB_API int GetMass();

	VICARLIB_API void RotationTestWithSign();
}