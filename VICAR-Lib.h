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
#define SET_R_CHANGE_OF_SPEED_WITH_SIGN 160



using namespace std;


void SetPositionAndSpeedMode();
// Reset all the Control Word
void ResetControlWord();

// test functions
void RotationTestWithSign();
// set drive parameters
void SetParametersX(uint16_t kp, uint16_t ki, uint16_t reference_filter);
void SetParametersY(uint16_t kp, uint16_t ki, uint16_t reference_filter);
void SetParametersR(uint16_t kp, uint16_t ki, uint16_t reference_filter);

// check if the system is moving
bool IsMoving();
// check if the system is ready
bool IsReady();

// set R position
void SetPositionR(float pos_r);

// get X,Y positions
float* GetPosition();
float GetPositionX();
float GetPositionY();
float GetPositionR();
// set X,Y positions
void SetPositionXY(int pos_x, int pos_y, int speed_x, int speed_y);

void SetSpeedWithSignR(int speed);

// get velocity
float* GetVelocity();
float GetVelocityX();
float GetVelocityY();
float GetVelocityR();
// get current from X,Y
short* GetCurrent();
short GetCurrentX();
short GetCurrentY();
short GetCurrentR();

// get drive parameters
uint16_t* GetParametersX();
uint16_t* GetParametersY();
uint16_t* GetParametersR();






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
	VICARLIB_API int GetMass();
	// calculate force to apply
	VICARLIB_API float GetForceX();
	VICARLIB_API float GetForceY();
	VICARLIB_API float GetAbsoluteAngle();
	VICARLIB_API float GetTorque();

	/*************************************************/

	//TODO THE FOLLOWING FUNCTIONS HAVE TO BE PRIVATE
	//just a test function
	VICARLIB_API int Test();
}