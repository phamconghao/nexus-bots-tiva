
/*
V1.1	201104
1. motorwheel library version 1.1,compatible with maple.

V1.1.1	201110
1. Add Acceleration

V1.2	201207
1. Double CPR from 12 to 24, Interrupt Type from RISING to CHANGE.
2. Reduce default sample time from 10ms to 5ms.
3. Compatible with Namiki 22CL-3501PG80:1, by "#define _NAMIKI_MOTOR" before "#include ...".

V1.3	201209
1. R2WD::delayMS(), Omni3WD::delayMS(), Omni4WD::delayMS() are re-implemented, more exactly.
2. getSpeedRPM(), setSpeedRPM(), getSpeedMMPS(), setSpeedMMPS() are plug-minus/direction sensitive now.
3. Acceleration is disabled.

V1.4	201209	Not Released
1. Increase CPR from 24 to 48 for Faulhaber 2342.

V1.5	201209	Omni4WD is re-implemented, and now return value of Omni4WD::getSpeedMMPS() is correct.

 */

#ifndef MotorWheel_H
#define MotorWheel_H

// #define DEBUG

#include <Arduino.h>
#include <pid_lib.h>
#include "../../include/debug_printf.h"

#define MAX_PWM 			255

#define DIR_ADVANCE 		HIGH
#define DIR_BACKOFF 		LOW

#define PIN_UNDEFINED 		255
#define REF_VOLT 			12

#ifdef _NAMIKI_MOTOR
	#define TRIGGER 		CHANGE
	#define CPR 			4 // Namiki motor
	#define DIR_INVERSE
	#define REDUCTION_RATIO 80
#else
	#define TRIGGER 		RISING
	#define CPR 			12 // Faulhaber motor
	#define DIR_INVERSE 	!
	#define REDUCTION_RATIO 64
#endif

#define MAX_SPEEDRPM 		8000

#define SEC_PER_MIN 		60
#define MICROS_PER_SEC 		1000000
#define SPEEDPPS2SPEEDRPM(freq) ((unsigned long)(freq) * (SEC_PER_MIN) / (CPR)) //(freq*SEC_PER_MIN/CPR)

#define KC 					0.31
#define TAUI 				0.02
#define TAUD 				0.00
#define SAMPLETIME 			5

#define Baudrate 			115200

#ifndef REDUCTION_RATIO
	#define REDUCTION_RATIO 64
#endif

#ifndef PI
	#define PI 				3.1416
#endif

#define CIRMM 				314 // mm

/**
 * @brief Encoder Params structure to hold all the relevant information for
 * 		  the encoder interrupt handling
 */
typedef struct Encoder_Params_t
{
	/* Pointer to interrupt handler fnx */
	void (*Encoder_Handler)();
	volatile long pulses;
	volatile unsigned long pulseStartMicros;
	volatile unsigned long pulseEndMicros;
	volatile unsigned int speedPPS;
	// volatile unsigned int  lastSpeedPPS;
	// volatile int accPPSS;	// acceleration, Pulse Per Sec^2
	volatile bool currDirection;
	unsigned char pinIRQB;
	unsigned char pinIRQ;
} Encoder_Params;


/** @class Motor
 *  @brief Implementation of Motor control based on PID algorithm. 
 * 		   Inheritance class of PID class
 */
class Motor : public PID
{
public:
	/**
	 *  @brief Motor control Standard Constructor function.
	 *
	 * 	@details Links the Motor controller to PID and initialize
	 *         	 GPIO peripheral, PWM for speed control and 
	 * 		     interrupt handler for encoders
	 *
	 *  @param pinPWM: ...
	 *  @param pinDir: ...
	 *  @param pinIRQ: ...
	 *  @param pinIRQB: ...
	 *  @param encoderParams: ...
	 *
	 *  @returns 	None
	 */
	Motor(  Encoder_Params_t *encoderParams, 
            unsigned char pinPWM, unsigned char pinDir,
		    unsigned char pinIRQ, unsigned char pinIRQB
		  );

	Encoder_Params_t* m_encoderParams;

	/**	
	 *  @brief Encoder Interrupt configuration
	 *
	 *  @details ...
	 *
	 *  @param None
	 *
	 *  @returns None
	 *
	 *  @retval None
	 */
	void setupInterrupt();

	/**	
	 *  @brief Get configured PWM Pin
	 *
	 *  @details ...
	 *
	 *  @param None
	 *
	 *  @returns Configured PWM Pin
	 *
	 *  @retval (unsigned char) Motor::pinPWM
	 */
	unsigned char getPinPWM() const;

	/**	
	 *  @brief Get configured Direction Pin
	 *
	 *  @details ...
	 *
	 *  @param None
	 *
	 *  @returns Configured Direction Pin
	 *
	 *  @retval (unsigned char) Motor::pinDir
	 */
	unsigned char getPinDir() const;

	/**	
	 *  @brief Get configured Interrupt Pin
	 *
	 *  @details ...
	 *
	 *  @param None
	 *
	 *  @returns Configured Interrupt Pin
	 *
	 *  @retval (unsigned char) Encoder_Params_t::pinIRQ
	 */
	unsigned char getPinIRQ() const;

	/**	
	 *  @brief Get configured Interrupt B Pin
	 *
	 *  @details ...
	 *
	 *  @param None
	 *
	 *  @returns Configured Interrupt B Pin
	 *
	 *  @retval (unsigned char) Encoder_Params_t::pinIRQB
	 */
	unsigned char getPinIRQB() const;

	/**	
	 *  @brief Run motor PWM with specified control PWM value and direction
	 *
	 *  @param PWM Control PWM value
	 *  @param dir Control direction
	 *  @param saveDir Option saving dir to Motor::desiredDirection (Default = true)
	 *
	 *  @returns Current PWM value
	 *
	 *  @retval (unsigned int) Motor::speedPWM
	 */
	unsigned int runPWM(unsigned int PWM, bool dir, bool saveDir = true);

	/**	
	 *  @brief Run motor PWM with ADVANCE direction
	 *
	 *  @details ...details
	 * 
	 *  @param PWM Control PWM value
	 *
	 *  @returns runPWM(PWM, DIR_ADVANCE)
	 * 
	 *  @retval (unsigned int) Motor::speedPWM
	 */
	unsigned int advancePWM(unsigned int PWM);

	/**	
	 *  @brief Run motor PWM with BACKOFF direction
	 *
	 *  @details ...details
	 *
	 *  @param PWM Control PWM value
	 *
	 *  @returns runPWM(PWM, DIR_BACKOFF)
	 * 
	 *  @retval (unsigned int) Motor::speedPWM
	 */
	unsigned int backoffPWM(unsigned int PWM);

	/**	
	 *  @brief Get current PWM control value
	 *
	 *  @details ...details
	 *
	 *  @param param
	 *
	 *  @returns Current PWM value
	 *
	 *  @retval (unsigned int) Motor::speedPWM
	 */
	unsigned int getPWM() const;
	
	/**	
	 *  @brief Set Desired motor control direction
	 *
	 *  @details ...details
	 *
	 *  @param dir Desired control direction
	 *
	 *  @returns getDesiredDir()
	 *
	 *  @retval (bool) Motor::desiredDirection
	 */
	bool setDesiredDir(bool dir);

	/**	
	 *  @brief Get Desired motor control direction
	 *
	 *  @details ...details
	 *
	 *  @param None
	 *
	 *  @returns Desired control direction
	 *
	 *  @retval (bool) Motor::desiredDirection
	 */
	bool getDesiredDir() const;

	/**	
	 *  @brief Reverse Desired motor control direction
	 *
	 *  @details ...details
	 *
	 *  @param None
	 *
	 *  @returns getDesiredDir()
	 *
	 *  @retval (bool) Motor::desiredDirection
	 */
	bool reverseDesiredDir();

	/**	
	 *  @brief Set Current motor control direction
	 *
	 *  @details ...details
	 *
	 *  @param None
	 *
	 *  @returns getDesiredDir()
	 *
	 *  @retval (bool) Motor::desiredDirection
	 */
	bool setCurrDir();

	/**	
	 *  @brief Get Desired motor control direction
	 *
	 *  @details ...details
	 *
	 *  @param None
	 *
	 *  @returns getDesiredDir()
	 *
	 *  @retval (bool) Motor::desiredDirection
	 */
	bool getCurrDir() const;

	// int getAccRPMM() const;		// Acceleration, Round Per Min^2

	/**	
	 *  @brief Get current motor speed in RPM unit from speed in PPS unit
	 *
	 *  @details ...details
	 *
	 *  @param None
	 *
	 *  @returns Current speed in RPM unit
	 *
	 *  @retval (volatile unsigned int) SPEEDPPS2SPEEDRPM(Encoder_Params_t::speedPPS)
	 */
	int getSpeedRPM() const;

	/**	
	 *  @brief Set motor speed in RPM with specified speed and direction
	 *
	 *  @details ...details
	 *
	 *  @param speedRPM Desired motor speed (RPM)
	 *  @param dir Desired motor direction
	 *
	 *  @returns Absolute value of Current speed in RPM unit
	 *
	 *  @retval (volatile unsigned int) SPEEDPPS2SPEEDRPM(Encoder_Params_t::speedPPS)
	 */
	unsigned int setSpeedRPM(int speedRPM, bool dir); // preserve

	/**	
	 *  @brief Set motor speed in RPM with specified speed only
	 *
	 *  @details ...details
	 *
	 *  @param speedRPM Desired motor speed (RPM)
	 *
	 *  @returns Absolute value of Current speed in RPM unit
	 *
	 *  @retval (volatile unsigned int) SPEEDPPS2SPEEDRPM(Encoder_Params_t::speedPPS)
	 */
	int setSpeedRPM(int speedRPM);

	/**	
	 *  @brief Initialize PID parameters
	 *
	 *  @details ...details
	 *
     *  @param kc           Controller gain, a tuning parameter
     *  @param taui         Reset time, a tuning parameter
     *  @param taud         Derivative time, a tuning parameter
	 *  @param sampleTime   The period, in milliseconds, with which we want the the PID
     *                      calculation to occur (Default = 1000ms)
	 *
	 *  @returns The flag reveal that the setup task is success or not
	 *
	 *  @retval [true]  if PID setup successfully
	 *  @retval [false] if PID setup fail
	 */
	bool PIDSetup(float kc = KC, float taui = TAUI, float taud = TAUD, unsigned int sampleTime = 1000);

	/**	
	 *  @brief Get the PID current status, was Enabled or not
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns The pidCtrl flag
	 *
	 *  @retval [true]  if PID was Enabled
	 *  @retval [false] if PID was Disabled
	 */
	bool PIDGetStatus() const;

	/**	
	 *  @brief Enable the PID calculation
	 *
	 *  @details ...details
	 *
     *  @param kc           Controller gain, a tuning parameter
     *  @param taui         Reset time, a tuning parameter
     *  @param taud         Derivative time, a tuning parameter
	 *  @param sampleTime   The period, in milliseconds, with which we want the the PID
     *                      calculation to occur (Default = 1000ms)
	 *
	 *  @returns The pidCtrl flag
	 *
	 *  @retval [true]  if PID was Enabled
	 *  @retval [false] if PID was Disabled
	 */
	bool PIDEnable(float kc = KC, float taui = TAUI, float taud = TAUD, unsigned int sampleTime = 1000);

	/**	
	 *  @brief Disable the PID calculation
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns The pidCtrl flag
	 *
	 *  @retval [true]  if PID was Enabled
	 *  @retval [false] if PID was Disabled
	 */
	bool PIDDisable();

	/**	
	 *  @brief Reset the PID calculation
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns The Reset status
	 *
	 *  @retval [true]  if PID was Reset
	 */
	bool PIDReset();

	/**	
	 *  @brief Perform PID regulation process
	 *
	 *  @details ...details
	 *
     *  @param doRegulate Regulate option, [true] do Regulate / [false] do NOT regulate
	 *
	 *  @returns The Regulation status
	 *
	 *  @retval [true]  if PID was regulated
	 */
	bool PIDRegulate(bool doRegulate = true);

	/**	
	 *  @brief Set PID desired motor speed in RPM unit
	 *
	 *  @details ...details
	 *
     *  @param speedRPM PID desired motor speed in RPM unit
	 *
	 *  @returns Desired speed in RPM (Round per Min) unit
	 *
	 *  @retval (int) Motor::speedRPMDesired
	 */
	unsigned int PIDSetSpeedRPMDesired(unsigned int speedRPM);

	/**	
	 *  @brief Get PID desired motor speed in RPM unit
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns Desired speed in RPM (Round per Min) unit
	 *
	 *  @retval (int) Motor::speedRPMDesired
	 */
	unsigned int PIDGetSpeedRPMDesired() const;

	// int getAccPPSS() const;

	/**	
	 *  @brief Get current motor speed in PPS unit
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns Current motor speed in PPS (Pulse per Second) unit
	 *
	 *  @retval (volatile unsigned int) Encoder_Params_t::speedPPS
	 */
	int getSpeedPPS() const;

	/**	
	 *  @brief Set current pulse value of encoder
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns Motor::getCurrPulse()
	 *
	 *  @retval (volatile long) Encoder_Params_t::pulses
	 */
	long setCurrPulse(long _pulse);

	/**	
	 *  @brief Get current pulse value
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns The current pulse value of encoder
	 *
	 *  @retval (volatile long) Encoder_Params_t::pulses
	 */
	long getCurrPulse() const;

	/**	
	 *  @brief Reset current pulse value of encoder
	 *
	 *  @details ...details
	 *
     *  @param None
	 *
	 *  @returns Motor::setCurrPulse(0)
	 *
	 *  @retval (volatile long) Encoder_Params_t::pulses
	 */
	long resetCurrPulse();

	/**	
	 *  @brief Create delay in Milisecond unit
	 *
	 *  @details ...details
	 *
     *  @param ms Delay time in Milisecond
     *  @param debug Debug option
	 *
	 *  @returns None
	 */
	void delayMS(unsigned int ms, bool debug = false);

	void debugger() const;

private:
	/* PWM control GPIO pin */
	unsigned char m_pinPWM;
	/* Direction control GPIO pin */
	unsigned char m_pinDir;

	// bool currDirection;		// current direction

	/* Desired direction */
	bool desiredDirection;
	/* Current PWM value */
	unsigned int speedPWM;
	/* Input speed in RPM (Round per Min) unit */
	int speedRPMInput;
	/* Output speed in RPM (Round per Min) unit */
	int speedRPMOutput;
	/* Desired speed in RPM (Round per Min) unit */
	int speedRPMDesired;
	// float PWMEC;
	float speed2DutyCycle;

    /* The flag reveal that the PID was Enabled or NOT */
	bool pidCtrl;

	Motor();
};

/** @class GearedMotor
 *  @brief Implementation of Nexus geared motor control class with PID algorithm.
 * 		   Inheritance class of Motor class
 */
class GearedMotor : public Motor
{
public:
	GearedMotor(unsigned char pinPWM, unsigned char pinDir,
				unsigned char pinIRQ, unsigned char pinIRQB,
				Encoder_Params_t *encoderParams,
				unsigned int _ratio = REDUCTION_RATIO);
	// float getGearedAccRPMM() const;		// Acceleration, Round Per Min^2
	float getGearedSpeedRPM() const;
	float setGearedSpeedRPM(float gearedSpeedRPM, bool dir);
	// direction sensitive 201208
	float setGearedSpeedRPM(float gearedSpeedRPM);
	unsigned int getRatio() const;
	unsigned int setRatio(unsigned int ratio = REDUCTION_RATIO);

private:
	unsigned int _ratio;
};

/** @class MotorWheel
 *  @brief Implementation of Nexus motor wheel control class with PID algorithm.
 * 		   Inheritance class of GearedMotor class
 */
class MotorWheel : public GearedMotor
{ //
public:
	MotorWheel(unsigned char pinPWM, unsigned char pinDir,
			   unsigned char pinIRQ, unsigned char pinIRQB,
			   Encoder_Params_t *encoderParams,
			   unsigned int ratio = REDUCTION_RATIO, unsigned int cirMM = CIRMM);

	unsigned int getCirMM() const;
	unsigned int setCirMM(unsigned int cirMM = CIRMM);

	// direction sensitive 201208
	// int getAccCMPMM() const;	// Acceleration, CM Per Min^2
	int getSpeedCMPM() const;					 // cm/min
	int setSpeedCMPM(unsigned int cm, bool dir); // preserve
	int setSpeedCMPM(int cm);
	// int getAccMMPSS() const;	// Acceleration, MM Per Sec^2
	int getSpeedMMPS() const;					 // mm/s
	int setSpeedMMPS(unsigned int mm, bool dir); // preserve
	int setSpeedMMPS(int mm);

private:
	unsigned int _cirMM;
};

#endif
