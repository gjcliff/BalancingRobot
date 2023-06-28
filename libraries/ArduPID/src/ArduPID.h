#pragma once
#include "Arduino.h"
#include "FireTimer.h"




const byte PRINT_INPUT    = B1;
const byte PRINT_OUTPUT   = B10;
const byte PRINT_SETPOINT = B100;
const byte PRINT_BIAS     = B1000;
const byte PRINT_P        = B10000;
const byte PRINT_I        = B100000;
const byte PRINT_D        = B1000000;




enum pOn { P_ON_E, P_ON_M };
enum mode { OFF, ON };
enum dir { FORWARD, BACKWARD };




class ArduPID
{
public:
	float* input;
	float* output;
	float* setpoint;




	virtual void begin(float* _input,
		               float* _output,
		               float* _setpoint,
		               const float& _p = 0,
		               const float& _i = 0,
		               const float& _d = 0,
		               const pOn& _pOn = P_ON_E,
		               const dir& _direction = FORWARD,
		               const unsigned int& _minSamplePeriodMs = 0,
		               const float& _bias = 0);
	void start();
	void reset();
	void stop();
	virtual void compute();
	void setOutputLimits(const float& min, const float& max);
	void setWindUpLimits(const float& min, const float& max);
	void setDeadBand(const float& min, const float& max);
	void setPOn(const pOn& _pOn);
	void setBias(const float& _bias);
	void setCoefficients(const float& _p, const float& _i, const float& _d);
	void setDirection(const dir& _direction);
	void reverse();
	void setSampleTime(const unsigned int& _minSamplePeriodMs);

	float B();
	float P();
	float I();
	float D();
	
	void debug(Stream* stream = &Serial,
	           const char* controllerName = "controller",
		       const byte& mask = 0xFF);




protected:
	float bias;

	float outputMax = 255;
	float outputMin = 0;

	float windupMax = 1000;
	float windupMin = -1000;

	float deadBandMax = 0;
	float deadBandMin = 0;

	float curError;
	float curSetpoint;
	float curInput;

	float lastError;
	float lastSetpoint;
	float lastInput;

	float pIn;
	float iIn;
	float dIn;

	float kp;
	float ki;
	float kd;

	float pOut;
	float iOut;
	float dOut;

	pOn pOnType;
	mode modeType;
	dir direction;

	FireTimer timer;
};