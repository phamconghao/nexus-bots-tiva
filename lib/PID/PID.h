#ifndef PID_LIB_h
#define PID_LIB_h

#define AUTO    1
#define MANUAL  0
#define LIBRARY_VERSION 0.6

/** \class PID
 *  @brief Implementation of PID algorithm
 */
class PID
{
public:
    /**
     *  @brief PID Standard Constructor function.
     *         Links the PID to the Input, Output, and Setpoint.
     *         Initial tuning parameters are also set here.
     *
     * 	@details Constructor used by most users.  the parameters specified are those for
     * 	         for which we can't set up reliable defaults, so we need to have the user set them.
     *
     *  @param Input      PID input value
     *  @param Output     PID ouput value
     *  @param Setpoint   PID setpoint value
     *  @param Kc         Controller gain, a tuning parameter
     *  @param TauI       Reset time, a tuning parameter
     *  @param TauD       Derivative time, a tuning parameter
     *
     *  @returns None
     */
    PID(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD);

    /**
     *  @brief PID Overloaded Constructor function. If the user wants to implement
     *         feed-forward.
     *
     *  @details This one is for more advanced users.  it's essentially the same as the
     *           standard constructor, with one addition.  you can link to a Feed Forward bias,
     *           which lets you implement... um.. Feed Forward Control.  good stuff.
     *
     *  @param Input    PID input value
     *  @param Output   PID ouput value
     *  @param Setpoint PID setpoint value
     *  @param FFBias   Feed-Forward Bias value
     *  @param Kc       Controller gain, a tuning parameter
     *  @param TauI     Reset time, a tuning parameter
     *  @param TauD     Derivative time, a tuning parameter
     *
     *  @returns None
     */
    PID(int *Input, int *Output, int *Setpoint, int *FFBias, float Kc, float TauI, float TauD);

    /**
     *  @brief Set PID Input Limits. Tells the PID what 0-100% are for the Input
     *
     *	@details I don't see this function being called all that much (other than from the
     *           constructor.)  it needs to be here so we can tell the controller what it's
     *           input limits are, and in most cases the 0-1023 default should be fine.  if
     *           there's an application where the signal being fed to the controller is
     *           outside that range, well, then this function's here for you.
     *
     *  @param INMin The lower limitation of PID input
     *  @param INMax The upper limitation of PID input
     *
     *  @returns None
     */
    void SetInputLimits(int INMin, int INMax);

    /**
     *  @brief Set PID Output Limits. Tells the PID what 0-100% are for the Output
     *
     *  @details This function will be used far more often than SetInputLimits.  while
     *           the input to the controller will generally be in the 0-1023 range (which is
     *           the default already,)  the output will be a little different.  maybe they'll
     *           be doing a time window and will need 0-8000 or something.  or maybe they'll
     *           want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *           here.
     *
     *  @param OUTMin The lower limitation of PID output
     *  @param OUTMax The upper limitation of PID output
     *
     *  @returns None
     */
    void SetOutputLimits(int OUTMin, int OUTMax);

    /**
     *  @brief Set PID Tuning params. While most users will set the tunings once in the
     *         constructor, this function gives the user the option
     *         of changing tunings during runtime for Adaptive control
     *
     *  @details This function allows the controller's dynamic performance to be adjusted.
     *           it's called automatically from the constructor, but tunings can also
     *           be adjusted on the fly during normal operation
     *
     *  @param Kc   Controller gain, a tuning parameter
     *  @param TauI Reset time, a tuning parameter
     *  @param TauD Derivative time, a tuning parameter
     *
     *  @returns None
     */
    void SetTunings(float Kc, float TauI, float TauD);

    /**
     *  @brief Reset PID. Reinitializes controller internals, automatically
     *         called on a manual to auto transition
     *
     *  @details Do all the things that need to happen to ensure a bumpless transfer
     *           from manual to automatic mode.  this shouldn't have to be called from the
     *           outside. In practice though, it is sometimes helpful to start from scratch,
     *           so it was made publicly available
     *
     *  @param None
     *
     *  @returns None
     */
    void Reset();

    /**
     *  @brief Set PID Mode. Sets PID to either Manual (0) or Auto (non-0)
     *
     *  @details Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     *           when the transition from manual to auto occurs, the controller is
     *           automatically initialized
     *
     *  @param Mode The operation mode of PID. Manual [0] or Auto [non-0]
     *
     *  @returns None
     */
    void SetMode(int Mode);

    /**
     *  @brief Set PID Sampling time. Sets the frequency, in Milliseconds, with which
     *         the PID calculation is performed.  Default is 1000
     *
     *  @param NewSampleTime New sampling time
     *
     *  @returns 	None
     */
    void SetSampleTime(int NewSampleTime);

    /**
     *  @brief Compute PID. Performs the PID calculation. It should be
     *         called every time loop() cycles. ON/OFF and calculation frequency
     *         can be set using SetMode(), SetSampleTime() respectively
     *     
     *  @details This, as they say, is where the magic happens.  this function should be called
     *           every time "void loop()" executes.  the function will decide for itself whether a new
     *           pid Output needs to be computed
     *       
     *           Some notes for people familiar with the nuts and bolts of PID control:
     *           - I used the Ideal form of the PID equation.  mainly because I like IMC
     *             tunings.  lock in the I and D, and then just vary P to get more
     *             aggressive or conservative
     *           - While this controller presented to the outside world as being a Reset Time
     *             controller, when the user enters their tunings the I term is converted to
     *             Reset Rate.  I did this merely to avoid the div0 error when the user wants
     *             to turn Integral action off.
     *           - Derivative on Measurement is being used instead of Derivative on Error.  The
     *             performance is identical, with one notable exception.  DonE causes a kick in
     *             the controller output whenever there's a setpoint change. DonM does not.
     *       
     *           If none of the above made sense to you, and you would like it to, go to:
     *           @reqs http://www.controlguru.com
     *
     *  @param None
     *
     *  @returns None
     */
    void Compute();

    /**
     *  @brief Get PID calculation state.
     *
     *  @details In certain situations, it helps to know when the PID has
     *           computed this bit will be true for one cycle after the PID
     *           calculation has occurred
     *
     *  @param None
     *
     *  @return The status of justCalced flag that will tell the
     *          outside world that the output was just computed
     *
     *  @retval [1] Done
     *  @retval [0] NOT done yet
     */
    bool JustCalculated();

    /**
     *  @brief Get PID calculation mode.
     *
     *  @return The status of inAuto flag, current operation Mode of PID
     *
     *  @retval [1] Auto
     *  @retval [0] Manual
     */
    int GetMode();

    /**
     *  @brief Get PID minimum input level.
     *
     *  @return The value of minimum input level
     *
     *  @retval (int) PID::inMin
     */
    int GetINMin();

    /**
     *  @brief Get PID maximum input level.
     *
     *  @return The value of maximum input level
     *
     *  @retval (int) (PID::inMin + PID::inSpan)
     */
    int GetINMax();

    /**
     *  @brief Get PID minimum output level.
     *
     *  @return The value of minimum output level
     *
     *  @retval (int) PID::outMin
     */
    int GetOUTMin();

    /**
     *  @brief Get PID maximum output level.
     *
     *  @return The value of maximum output level
     *
     *  @retval (int) PID::outMax
     */
    int GetOUTMax();

    /**
     *  @brief Get PID Sampling time.
     *
     *  @return The value of tSample variable
     *
     *  @retval (int) PID::tSample
     */
    int GetSampleTime();

    /**
     *  @brief Get current (P)roportional Tuning Parameter.
     *
     *  @return The current value of kc coefficent
     *
     *  @retval (float) PID::P_Param
     */
    float GetP_Param();

    /**
     *  @brief Get current (I)ntegral Tuning Parameter.
     *
     *  @return The current value of taur coefficent
     *
     *  @retval (float) PID::I_Param
     */
    float GetI_Param();

    /**
     *  @brief Get current (D)erivative Tuning Parameter.
     *
     *  @return The current value of taud coefficent
     *
     *  @retval (float) PID::D_Param
     */
    float GetD_Param();

private:
    /**	@brief Common PID Constructor that is shared by the constructors.
     *
     *  @details Most of what is done in the two constructors is the same.  that code
     * 	         was put here for ease of maintenance and (minor) reduction of library size
     *
     *  @param Input      PID input value
     *  @param Output     PID ouput value
     *  @param Setpoint   PID setpoint value
     *  @param Kc         Controller gain, a tuning parameter
     *  @param TauI       Reset time, a tuning parameter
     *  @param TauD       Derivative time, a tuning parameter
     *
     *  @returns None
     */
    void ConstructorCommon(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD);

    /* (P)roportional Tuning Parameter, scaled, tweaked parameters we'll actually be using */
    float m_Kc;
    /* (I)ntegral Tuning Parameter, scaled, tweaked parameters we'll actually be using */
    float m_TauR;
    /* (D)erivative Tuning Parameter, scaled, tweaked parameters we'll actually be using */
    float m_TauD;

    float cof_A;
    float cof_B;
    float cof_C;

    /* (P)roportional Tuning Parameter, pretty parameters we'll give back to the user*/
    float P_Param;
    /* (I)ntegral Tuning Parameter, pretty parameters we'll give back to the user*/
    float I_Param;
    /* (D)erivative Tuning Parameter, pretty parameters we'll give back to the user*/
    float D_Param;

    // * Pointers to the Input, Output, and Setpoint variables. This creates a 
    //   hard link between the variables and the PID, freeing the user from having
    //   to constantly tell us what these values are.  with pointers we'll just know.
    /* Pointers to the Input variables */
    int *m_Input;
    /* Pointers to the Output variables */
    int *m_Output;
    /* Pointers to the Setpoint variables */
    int *m_Setpoint;

    /* Pointer to the External FeedForward bias, only used if the advanced constructor is used*/
    int *m_Bias;
    /* Internal flag that tells us if we're using FeedForward or not */
    bool UsingFeedForward;
    /* Figure out when the PID Calculation needs to be performed next to determine when to compute next */
    unsigned long nextCompTime;
    /* The period, in milliseconds, with which we want the the PID calculation to occur.*/
    unsigned long tSample;
    /* Flag letting us know if we are in Automatic or not */
    bool inAuto;

    //   the derivative required for the D term
    // float accError;              // * the (I)ntegral term is based on the sum of error over
                                    //   time.  this variable keeps track of that
    /* The base output from which the PID operates */
    float bias;

    /* The Error value */
    int Err;
    /* The last Error value */
    int lastErr;
    /* The previous Error value */
    int prevErr;

    /* The lower limitation of PID input */
    float inMin;
    float inSpan;
    /* The lower limitation of PID output */
    float outMin;
    float outSpan;

    /* Flag gets set for one cycle after the pid calculates */
    bool isCalculated;
};

#endif /* PID_LIB_H */
