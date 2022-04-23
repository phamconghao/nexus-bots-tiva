#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

#include <PID.h>
#include "fuzzy_table.h"
#include <wiring_private.h>
#include <HardwareSerial.h>

PID::PID(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{
	PID::ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);
	UsingFeedForward = false;
	PID::Reset();
}

PID::PID(int *Input, int *Output, int *Setpoint, int *FFBias, float Kc, float TauI, float TauD)
{
	PID::ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);
    /* tell the controller that we'll be using an external */
	UsingFeedForward = true;
    /* bias, and where to find it */
	m_Bias = FFBias;
	PID::Reset();
}

void PID::ConstructorCommon(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{
    // default the limits to the full ranges of the I/O
	PID::SetInputLimits(0, 1023); 
	PID::SetOutputLimits(0, 255);

	tSample = 1000; // default Controller Sample Time is 1 second

	PID::SetTunings(Kc, TauI, TauD);

	nextCompTime = millis();
	inAuto = false;
	m_Output = Output;
	m_Input = Input;
	m_Setpoint = Setpoint;

	Err = lastErr = prevErr = 0;
}

void PID::SetInputLimits(int INMin, int INMax)
{
	// after verifying that mins are smaller than maxes, set the values
	if (INMin >= INMax)
    {
		return;
    }

	inMin = INMin;
	inSpan = INMax - INMin;
}

void PID::SetOutputLimits(int OUTMin, int OUTMax)
{
	// after verifying that mins are smaller than maxes, set the values
	if (OUTMin >= OUTMax)
    {
		return;
    }

	outMin = OUTMin;
	outSpan = OUTMax - OUTMin;
}

void PID::SetTunings(float Kc, float TauI, float TauD)
{
	// verify that the tunings make sense
	if (Kc == 0.0 || TauI < 0.0 || TauD < 0.0)
    {
		return;
    }

	// we're going to do some funky things to the input numbers so all
	// our math works out, but we want to store the numbers intact
	// so we can return them to the user when asked.
	P_Param = Kc;
	I_Param = TauI;
	D_Param = TauD;

	// convert Reset Time into Reset Rate, and compensate for Calculation frequency
	float tSampleInSec = ((float)tSample / 1000.0);
	float tempTauR;

	if (TauI == 0.0)
    {
		tempTauR = 0.0;
    }
	else
    {
		tempTauR = (1.0 / TauI) * tSampleInSec;
    }

	m_Kc = Kc;
	m_TauR = tempTauR;
	m_TauD = TauD / tSampleInSec;

	cof_A = m_Kc * (1 + m_TauR + m_TauD);
	cof_B = m_Kc * (1 + 2 * m_TauD);
	cof_C = m_Kc * m_TauD;
}

void PID::Reset()
{
	if (UsingFeedForward)
	{
		bias = (*m_Bias - outMin) / outSpan;
	}
	else
	{
		bias = (*m_Output - outMin) / outSpan;
	}
}

void PID::SetMode(int Mode)
{
	if (Mode != 0 && !inAuto)
	{ 
        // we were in manual, and we just got set to auto.
		// reset the controller internals
		PID::Reset();
	}
	inAuto = (Mode != 0);
}

void PID::SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		// convert the time-based tunings to reflect this change
		m_TauR *= ((float)NewSampleTime) / ((float)tSample);
		m_TauD *= ((float)NewSampleTime) / ((float)tSample);
		tSample = (unsigned long)NewSampleTime;

		cof_A = m_Kc * (1 + m_TauR + m_TauD);
		cof_B = m_Kc * (1 + 2 * m_TauD);
		cof_C = m_Kc * m_TauD;
	}
}

void PID::Compute()
{
	isCalculated = false;
	if (!inAuto)
    {
        // Just leave if we're in manual mode
		return;
    }

	unsigned long now = millis();

	// millis() wraps around to 0 at some point.  depending on the version of the
	// Arduino Program you are using, it could be in 9 hours or 50 days.
	// this is not currently addressed by this algorithm.

	// Perform PID Computations if it's time...
	if (now >= nextCompTime)
	{
		Err = *m_Setpoint - *m_Input;
		// If we're using an external bias (i.e. the user used the
		// overloaded constructor,) then pull that in now
		if (UsingFeedForward)
		{
			bias = *m_Bias - outMin;
		}

        // Disables all interrupts, prevents interrupts from happpening
		noInterrupts();
		// Perform the PID calculation:
		//      output = bias + m_Kc * ((Err - lastErr) + (m_TauR * Err) + (m_TauD * (Err - 2*lastErr + prevErr)))
		int output = bias + (cof_A * Err - cof_B * lastErr + cof_C * prevErr);
        // Re-enable all interrupts
		interrupts();

		// Make sure the computed output is within output constraints
		if (output < -outSpan)
        {
			output = -outSpan;
        }
		else if (output > outSpan)
        {
			output = outSpan;
        }

        // Update errors
		prevErr = lastErr;
		lastErr = Err;

		// Scale the output from percent span back out to a real world number
		*m_Output = output;

        // Determine the next time the computation
		nextCompTime += tSample;
		if (nextCompTime < now)
        {
            nextCompTime = now + tSample; // should be performed
        }

        // Set the flag that will tell the outside world that the output was just computed
		isCalculated = true;
	}
}

/*****************************************************************************
 * STATUS SECTION
 * These functions allow the outside world to query the status of the PID
 *****************************************************************************/

bool PID::JustCalculated()
{
	return isCalculated;
}

int PID::GetMode()
{
	if (inAuto)
		return 1;
	else
		return 0;
}

int PID::GetINMin()
{
	return inMin;
}

int PID::GetINMax()
{
	return inMin + inSpan;
}

int PID::GetOUTMin()
{
	return outMin;
}

int PID::GetOUTMax()
{
	return outMin + outSpan;
}

int PID::GetSampleTime()
{
	return tSample;
}

float PID::GetP_Param()
{
	return P_Param;
}

float PID::GetI_Param()
{
	return I_Param;
}

float PID::GetD_Param()
{
	return D_Param;
}
