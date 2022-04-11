#include <MotorWheel.h>

Motor::Motor(   Encoder_Params_t *encoderParams,
                unsigned char pinPWM, unsigned char pinDir,
			    unsigned char pinIRQ, unsigned char pinIRQB
			 )
	: PID(&speedRPMInput, &speedRPMOutput, &speedRPMDesired, KC, TAUI, TAUD),
	  m_encoderParams(encoderParams), m_pinPWM(pinPWM), m_pinDir(pinDir)
{
	m_encoderParams->pinIRQ = pinIRQ;
	m_encoderParams->pinIRQB = pinIRQB;

	/* GPIO Pin configuration */
	pinMode(m_pinPWM, OUTPUT);
	pinMode(m_pinDir, OUTPUT);
	/* Interrupt Pin configuration */
	pinMode(m_encoderParams->pinIRQ, INPUT);
	if (m_encoderParams->pinIRQB != PIN_UNDEFINED)
	{
		pinMode(m_encoderParams->pinIRQB, INPUT);
	}
	/* Disable PID */
	PIDDisable();
}

void Motor::setupInterrupt()
{
/*for maple*/
#if defined(BOARD_maple) || defined(BOARD_maple_native) || defined(BOARD_maple_mini)
	attachInterrupt(m_encoderParams->pinIRQ, m_encoderParams->Encoder_Handler, TRIGGER); // RISING --> CHANGE 201207

/*for arduino*/
#else
	attachInterrupt(m_encoderParams->pinIRQ - 2, m_encoderParams->Encoder_Handler, TRIGGER);
#endif
}

unsigned char Motor::getPinPWM() const
{
	return m_pinPWM;
}

unsigned char Motor::getPinDir() const
{
	return m_pinDir;
}

unsigned char Motor::getPinIRQ() const
{
	return m_encoderParams->pinIRQ;
}

unsigned char Motor::getPinIRQB() const
{
	return m_encoderParams->pinIRQB;
}

unsigned int Motor::runPWM(unsigned int PWM, bool dir, bool saveDir)
{
	/* Pass PWM value to Motor::speedPWM */
	speedPWM = PWM;
	/* Save dir to Motor::desiredDirection (saving for default) */
	if (saveDir)
	{
		desiredDirection = dir;
	}
	/* Write PWM and direction to GPIO */
	analogWrite(m_pinPWM, PWM);
	digitalWrite(m_pinDir, dir);
	return speedPWM;
}

unsigned int Motor::advancePWM(unsigned int PWM)
{
	return runPWM(PWM, DIR_ADVANCE);
}

unsigned int Motor::backoffPWM(unsigned int PWM)
{
	return runPWM(PWM, DIR_BACKOFF);
}

unsigned int Motor::getPWM() const
{
	return speedPWM;
}

bool Motor::setDesiredDir(bool dir)
{
	desiredDirection = dir;
	return getDesiredDir();
}

bool Motor::getDesiredDir() const
{
	return desiredDirection;
}

bool Motor::reverseDesiredDir()
{
	runPWM(getPWM(), !getDesiredDir());
	return getDesiredDir();
}

bool Motor::setCurrDir()
{
	if (getPinIRQB() != PIN_UNDEFINED)
	{
		return m_encoderParams->currDirection = digitalRead(m_encoderParams->pinIRQB);
	}
	return false;
}

bool Motor::getCurrDir() const
{
	return m_encoderParams->currDirection;
}

int Motor::getSpeedRPM() const
{
	if (getCurrDir() == DIR_ADVANCE)
	{
		return SPEEDPPS2SPEEDRPM(m_encoderParams->speedPPS);
	}
	return -SPEEDPPS2SPEEDRPM(m_encoderParams->speedPPS);
}

unsigned int Motor::setSpeedRPM(int speedRPM, bool dir)
{
	PIDSetSpeedRPMDesired(speedRPM);
	setDesiredDir(dir);
	return abs(getSpeedRPM());
}

int Motor::setSpeedRPM(int speedRPM)
{
	if (speedRPM >= 0)
	{
		return setSpeedRPM(speedRPM, DIR_ADVANCE);
	}
	else
	{
		return setSpeedRPM(abs(speedRPM), DIR_BACKOFF);
	}
}

bool Motor::PIDSetup(float kc, float taui, float taud, unsigned int sampleTime)
{  
	PID::SetTunings(kc, taui, taud);
	PID::SetInputLimits(0, MAX_SPEEDRPM);
	PID::SetOutputLimits(0, MAX_SPEEDRPM);
	PID::SetSampleTime(sampleTime);
	PID::SetMode(AUTO);
	return true;
}

bool Motor::PIDGetStatus() const
{
	return pidCtrl;
}

bool Motor::PIDEnable(float kc, float taui, float taud, unsigned int sampleTime)
{
	setupInterrupt();
	PIDSetup(kc, taui, taud, sampleTime);
	return pidCtrl = true;
}

bool Motor::PIDDisable()
{
	return pidCtrl = false;
}

bool Motor::PIDReset()
{
	if (PIDGetStatus() == false)
	{
		return false;
	}
	PID::Reset();
	return true;
}

bool Motor::PIDRegulate(bool doRegulate)
{
	if (PIDGetStatus() == false)
	{
		return false;
	}
	if (getPinIRQB() != PIN_UNDEFINED && getDesiredDir() != getCurrDir())
	{
		speedRPMInput = -SPEEDPPS2SPEEDRPM(m_encoderParams->speedPPS);
	}
	else
	{
		speedRPMInput = SPEEDPPS2SPEEDRPM(m_encoderParams->speedPPS);
	}

	PID::Compute();
	if (doRegulate && PID::JustCalculated())
	{
		speed2DutyCycle += speedRPMOutput;

		if (speed2DutyCycle >= MAX_SPEEDRPM)
		{
			speed2DutyCycle = MAX_SPEEDRPM;
		}
		else if (speed2DutyCycle <= -MAX_SPEEDRPM)
		{
			speed2DutyCycle = -MAX_SPEEDRPM;
		}
		if (speed2DutyCycle >= 0)
		{
			runPWM(map(speed2DutyCycle, 0, MAX_SPEEDRPM, 0, MAX_PWM), getDesiredDir(), false);
		}
		else
		{
			runPWM(map(abs(speed2DutyCycle), 0, MAX_SPEEDRPM, 0, MAX_PWM), !getDesiredDir(), false);
		}
		return true;
	}
	return false;
}

unsigned int Motor::PIDSetSpeedRPMDesired(unsigned int speedRPM)
{
	if (speedRPM > MAX_SPEEDRPM)
	{
		speedRPMDesired = MAX_SPEEDRPM;
	}
	else
	{
		speedRPMDesired = speedRPM;
	}
	return PIDGetSpeedRPMDesired();
}

unsigned int Motor::PIDGetSpeedRPMDesired() const
{
	return speedRPMDesired;
}

int Motor::getSpeedPPS() const
{
	return m_encoderParams->speedPPS;
}

long Motor::getCurrPulse() const
{
	return m_encoderParams->pulses;
}

long Motor::setCurrPulse(long _pulse)
{
	m_encoderParams->pulses = _pulse;
	return getCurrPulse();
}

long Motor::resetCurrPulse()
{
	return setCurrPulse(0);
}

void Motor::delayMS(unsigned int ms, bool debug)
{
	for (unsigned long endTime = millis() + ms; millis() < endTime;)
	{
		PIDRegulate();
		if (debug && (millis() % 500 == 0))
		{
			debugger();
		}
		if (endTime - millis() >= SAMPLETIME)
		{
			delay(SAMPLETIME);
		}
		else
		{
			delay(endTime - millis());
		}
	}
}

void Motor::debugger() const
{

#ifdef DEBUG
		if (!Serial.available()) Serial.begin(Baudrate);
	/*
		Serial.print("m_pinPWM -> ");
		Serial.println(m_pinPWM,DEC);
		Serial.print("m_pinDir -> ");
		Serial.println(m_pinDir,DEC);
		Serial.print("pinIRQ -> ");
		Serial.println(pinIRQ,DEC);
		Serial.print("pinIRQB-> ");
		Serial.println(pinIRQB,DEC);
	 */

	Serial.print("DesiredDir -> ");
	Serial.println(desiredDirection);
	Serial.print("currDir ->");
	Serial.println(m_encoderParams->currDirection);

	Serial.print("PWM    -> ");
	Serial.println(speedPWM, DEC);
	Serial.print("Input  -> ");
	Serial.println(speedRPMInput, DEC);
	Serial.print("Output -> ");
	Serial.println(speedRPMOutput, DEC);
	Serial.print("Desired-> ");
	Serial.println(speedRPMDesired, DEC);

	/*
		Serial.print("speed2DutyCycle-> ");
		Serial.println(speed2DutyCycle);
		Serial.print("speedPPS> ");
		Serial.println(m_encoderParams->speedPPS,DEC);
		Serial.print("pulses -> ");
		Serial.println(m_encoderParams->pulses,DEC);
	 */

#endif
}

GearedMotor::GearedMotor(unsigned char pinPWM, unsigned char pinDir,
						 unsigned char pinIRQ, unsigned char pinIRQB,
						 Encoder_Params_t *encoderParams, unsigned int ratio) 
            : Motor(encoderParams, pinPWM, pinDir, pinIRQ, pinIRQB), _ratio(ratio)
{
	;
}

unsigned int GearedMotor::getRatio() const
{
	return _ratio;
}

unsigned int GearedMotor::setRatio(unsigned int ratio)
{
	_ratio = ratio;
	return getRatio();
}

float GearedMotor::getGearedSpeedRPM() const
{
	return (float)Motor::getSpeedRPM() / getRatio();
}

float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM, bool dir)
{
	// Motor::setSpeedRPM(abs(gearedSpeedRPM*REDUCTION_RATIO),dir);
	Motor::setSpeedRPM(abs(round(gearedSpeedRPM * _ratio)), dir);
	return getGearedSpeedRPM();
}

float GearedMotor::setGearedSpeedRPM(float gearedSpeedRPM)
{
	Motor::setSpeedRPM(round(gearedSpeedRPM * _ratio));
	return getGearedSpeedRPM();
}

MotorWheel::MotorWheel(unsigned char pinPWM, unsigned char pinDir,
					   unsigned char pinIRQ, unsigned char pinIRQB,
					   Encoder_Params_t *encoderParams,
					   unsigned int ratio, unsigned int cirMM) 
            : GearedMotor(pinPWM, pinDir, pinIRQ, pinIRQB, encoderParams, ratio), _cirMM(cirMM)
{
	;
}
unsigned int MotorWheel::getCirMM() const
{
	return _cirMM;
}
unsigned int MotorWheel::setCirMM(unsigned int cirMM)
{
	if (cirMM > 0)
		_cirMM = cirMM;
	return getCirMM();
}

/*
int MotorWheel::getAccCMPMM() const {
	debug();
	return int(GearedMotor::getGearedAccRPMM()*getCirMM()/10);
}
 */

int MotorWheel::getSpeedCMPM() const
{
	return int(GearedMotor::getGearedSpeedRPM() * getCirMM() / 10);
}

int MotorWheel::setSpeedCMPM(unsigned int cm, bool dir)
{
	GearedMotor::setGearedSpeedRPM(cm * 10.0 / getCirMM(), dir);
	return getSpeedCMPM();
}

int MotorWheel::setSpeedCMPM(int cm)
{
	// GearedMotor::setGearedSpeedRPM(cm/CIR,dir);
	GearedMotor::setGearedSpeedRPM(cm * 10.0 / getCirMM());
	return getSpeedCMPM();
}

/*
int MotorWheel::getAccMMPSS() const {
	debug();
	return int(getAccCMPMM()/6);
}
 */

int MotorWheel::getSpeedMMPS() const
{
	return int(getSpeedCMPM() / 6); //(mm/sec)/(cm/min) = 6
}

int MotorWheel::setSpeedMMPS(unsigned int mm, bool dir)
{
	setSpeedCMPM(mm * 6, dir);
	return getSpeedMMPS();
}

// direction sensitive, 201208
int MotorWheel::setSpeedMMPS(int mm)
{
	setSpeedCMPM(mm * 6);
	return getSpeedMMPS();
}
