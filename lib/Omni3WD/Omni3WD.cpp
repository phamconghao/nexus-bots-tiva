#include <Omni3WD.h>

Omni3WD::Omni3WD(MotorWheel *wheelBack, MotorWheel *wheelRight, MotorWheel *wheelLeft) 
    : m_wheelBack(wheelBack), m_wheelRight(wheelRight), m_wheelLeft(wheelLeft)
{
	setSwitchMotorsStat(MOTORS_SWITCH_STAT::BRL);
}

unsigned char Omni3WD::getSwitchMotorsStat() const
{
	return m_switchMotorsStat;
}

unsigned char Omni3WD::setSwitchMotorsStat(unsigned char switchMotorsStat)
{
	if ((MOTORS_SWITCH_STAT::BRL <= switchMotorsStat) && (switchMotorsStat <= MOTORS_SWITCH_STAT::RLB))
		m_switchMotorsStat = switchMotorsStat;
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::switchMotorsLeft()
{ 
    // --> BRL --> LBR --> RLB
	if (getSwitchMotorsStat() == MOTORS_SWITCH_STAT::RLB)
		setSwitchMotorsStat(MOTORS_SWITCH_STAT::BRL);
	else
		setSwitchMotorsStat(getSwitchMotorsStat() + 1);
	MotorWheel *temp = m_wheelBack;
	m_wheelBack = m_wheelRight;
	m_wheelRight = m_wheelLeft;
	m_wheelLeft = temp;
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::switchMotorsRight()
{ 
    // <-- BRL <-- LBR <--RLB
	if (getSwitchMotorsStat() == MOTORS_SWITCH_STAT::BRL)
		setSwitchMotorsStat(MOTORS_SWITCH_STAT::RLB);
	else
		setSwitchMotorsStat(getSwitchMotorsStat() - 1);
	MotorWheel *temp = m_wheelBack;
	m_wheelBack = m_wheelLeft;
	m_wheelLeft = m_wheelRight;
	m_wheelRight = temp;
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::switchMotorsReset()
{
	while (getSwitchMotorsStat() != MOTORS_SWITCH_STAT::BRL)
	{
		switchMotorsLeft();
	}
	return getSwitchMotorsStat();
}

unsigned char Omni3WD::getCarStat() const
{
	return m_carStat;
}

unsigned char Omni3WD::setCarStat(unsigned char carStat)
{
	if ((MOVEMENT_STAT::UNKNOWN <= carStat) && carStat <= MOVEMENT_STAT::ROTATERIGHT)
		return m_carStat = carStat;
	else
		return MOVEMENT_STAT::UNKNOWN;
}

unsigned int Omni3WD::setMotorAll(unsigned int speedMMPS, bool dir)
{
	wheelBackSetSpeedMMPS(speedMMPS, dir);
	wheelRightSetSpeedMMPS(speedMMPS, dir);
	wheelLeftSetSpeedMMPS(speedMMPS, dir);
	return wheelBackGetSpeedMMPS();
}

unsigned int Omni3WD::setMotorAllStop()
{
	return setMotorAll(0, DIR_ADVANCE);
}

unsigned int Omni3WD::setMotorAllAdvance(unsigned int speedMMPS)
{
	return setMotorAll(speedMMPS, DIR_ADVANCE);
}

unsigned int Omni3WD::setMotorAllBackoff(unsigned int speedMMPS)
{
	return setMotorAll(speedMMPS, DIR_BACKOFF);
}

unsigned int Omni3WD::setCarStop()
{
	setCarStat(MOVEMENT_STAT::STOP);
	return setMotorAll(0, DIR_ADVANCE);
}

unsigned int Omni3WD::setCarAdvance(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::ADVANCE);
	wheelBackSetSpeedMMPS(0, DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS, DIR_BACKOFF);
	wheelLeftSetSpeedMMPS(speedMMPS, DIR_ADVANCE);
	return wheelRightGetSpeedMMPS();
}

unsigned int Omni3WD::setCarBackoff(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::BACKOFF);
	wheelBackSetSpeedMMPS(0, DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS, DIR_ADVANCE);
	wheelLeftSetSpeedMMPS(speedMMPS, DIR_BACKOFF);
	return wheelRightGetSpeedMMPS();
}

unsigned int Omni3WD::setCarLeft(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::LEFT);
	wheelBackSetSpeedMMPS(speedMMPS, DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPS >> 1, DIR_BACKOFF);
	wheelLeftSetSpeedMMPS(speedMMPS >> 1, DIR_BACKOFF);
	return wheelBackGetSpeedMMPS();
}

unsigned int Omni3WD::setCarRight(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::RIGHT);
	wheelBackSetSpeedMMPS(speedMMPS, DIR_BACKOFF);
	wheelRightSetSpeedMMPS(speedMMPS >> 1, DIR_ADVANCE);
	wheelLeftSetSpeedMMPS(speedMMPS >> 1, DIR_ADVANCE);
	return wheelBackGetSpeedMMPS();
}

unsigned int Omni3WD::setCarRotateLeft(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::ROTATELEFT);
	return setMotorAllBackoff(speedMMPS);
}

unsigned int Omni3WD::setCarRotateRight(unsigned int speedMMPS)
{
	setCarStat(MOVEMENT_STAT::ROTATERIGHT);
	return setMotorAllAdvance(speedMMPS);
}

unsigned int Omni3WD::getCarSpeedMMPS() const
{
	unsigned int speedMMPS = wheelBackGetSpeedMMPS();
	if ((unsigned int)wheelRightGetSpeedMMPS() > speedMMPS)
	{
		speedMMPS = wheelRightGetSpeedMMPS();
	}
	if ((unsigned int)wheelLeftGetSpeedMMPS() > speedMMPS)
	{
		speedMMPS = wheelLeftGetSpeedMMPS();
	}
	return speedMMPS;
}

unsigned int Omni3WD::setCarSpeedMMPS(unsigned int speedMMPS, unsigned int ms)
{
	unsigned int carStat = getCarStat();
	int currSpeed = getCarSpeedMMPS(); //
	int speedTemp = speedMMPS;
	// unsigned int (Omni3WD::*carAction)(unsigned int speedMMPS);
	switch (carStat)
	{
        case MOVEMENT_STAT::UNKNOWN: // no break here
        case MOVEMENT_STAT::STOP:
            return currSpeed;
        case MOVEMENT_STAT::ADVANCE:
            carAction = &Omni3WD::setCarAdvance;
            break;
        case MOVEMENT_STAT::BACKOFF:
            carAction = &Omni3WD::setCarBackoff;
            break;
        case MOVEMENT_STAT::LEFT:
            carAction = &Omni3WD::setCarLeft;
            break;
        case MOVEMENT_STAT::RIGHT:
            carAction = &Omni3WD::setCarRight;
            break;
        case MOVEMENT_STAT::ROTATELEFT:
            carAction = &Omni3WD::setCarRotateLeft;
            break;
        case MOVEMENT_STAT::ROTATERIGHT:
            carAction = &Omni3WD::setCarRotateRight;
            break;
	}

	if (ms < 100 || abs(speedTemp - currSpeed) < 10)
	{
		(this->*carAction)(speedMMPS);
		return getCarSpeedMMPS();
	}

	for (int time = 0, speed = currSpeed; (unsigned int)time <= ms; time += 50)
	{
		speed = abs(map(time, 0, ms, currSpeed, speedTemp));
		(this->*carAction)(speed);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}

unsigned int Omni3WD::setCarSlow2Stop(unsigned int ms)
{
	return setCarSpeedMMPS(0, ms);
}

unsigned int Omni3WD::wheelBackSetSpeedMMPS(unsigned int speedMMPS, bool dir)
{
	return m_wheelBack->setSpeedMMPS(speedMMPS, dir);
}

int Omni3WD::wheelBackGetSpeedMMPS() const
{
	return m_wheelBack->getSpeedMMPS();
}

unsigned int Omni3WD::wheelRightSetSpeedMMPS(unsigned int speedMMPS, bool dir)
{
	return m_wheelRight->setSpeedMMPS(speedMMPS, dir);
}

int Omni3WD::wheelRightGetSpeedMMPS() const
{
	return m_wheelRight->getSpeedMMPS();
}

unsigned int Omni3WD::wheelLeftSetSpeedMMPS(unsigned int speedMMPS, bool dir)
{
	return m_wheelLeft->setSpeedMMPS(speedMMPS, dir);
}

int Omni3WD::wheelLeftGetSpeedMMPS() const
{
	return m_wheelLeft->getSpeedMMPS();
}

bool Omni3WD::PIDEnable(float kc, float taui, float taud, unsigned int interval)
{
	return m_wheelBack->PIDEnable(kc, taui, taud, interval) &&
		   m_wheelRight->PIDEnable(kc, taui, taud, interval) &&
		   m_wheelLeft->PIDEnable(kc, taui, taud, interval);
}

bool Omni3WD::PIDRegulate()
{
	return m_wheelBack->PIDRegulate() && m_wheelRight->PIDRegulate() && m_wheelLeft->PIDRegulate();
}

void Omni3WD::delayMS(unsigned int ms, bool debug)
{
	for (unsigned long endTime = millis() + ms; millis() < endTime;)
	{
		PIDRegulate();
		if (debug && (millis() % 500 == 0))
			debugger();
		if (endTime - millis() >= SAMPLETIME)
			delay(SAMPLETIME);
		else
			delay(endTime - millis());
	}
}

// new one
void Omni3WD::demoActions(unsigned int speedMMPS, unsigned int duration, unsigned int uptime, bool debug)
{
	unsigned int (Omni3WD::*carAction[])(unsigned int speedMMPS) = {
		&Omni3WD::setCarAdvance,
		&Omni3WD::setCarBackoff,
		&Omni3WD::setCarLeft,
		&Omni3WD::setCarRight,
		&Omni3WD::setCarRotateLeft,
		&Omni3WD::setCarRotateRight };

	for (int i = 0; i < 6; ++i)
	{
		(this->*carAction[i])(speedMMPS);
		setCarSpeedMMPS(speedMMPS, uptime);
		delayMS(duration, debug);
		setCarSlow2Stop(uptime);
	}

	setCarStop();
	delayMS(duration, debug);
	// switchMotorsLeft();
}

// original
void Omni3WD::demoActions_Orginal(unsigned int speedMMPS, unsigned int ms, bool debug)
{
	setCarAdvance(speedMMPS);
	delayMS(ms, debug);
	setCarBackoff(speedMMPS);
	delayMS(ms, debug);
	setCarLeft(speedMMPS);
	delayMS(ms, debug);
	setCarRight(speedMMPS);
	delayMS(ms, debug);
	setCarRotateLeft(speedMMPS);
	delayMS(ms, debug);
	setCarRotateRight(speedMMPS);
	delayMS(ms, debug);
	setCarStop();
	delayMS(1000, debug);
	switchMotorsLeft();
}

void Omni3WD::debugger(bool wheelBackDebug, bool wheelRightDebug, bool wheelLeftDebug) const
{
	if (wheelBackDebug)
		m_wheelBack->debugger();
	if (wheelRightDebug)
		m_wheelRight->debugger();
	if (wheelLeftDebug)
		m_wheelLeft->debugger();
}
