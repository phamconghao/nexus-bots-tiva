#include <MotorWheel.h>

/*
    wheelLeft	wheelRight
        //			\\



             ==
        wheelBack
 */

#ifndef Omni3WD_H
#define Omni3WD_H

enum MOVEMENT_STAT
{
    UNKNOWN,
    STOP,
    ADVANCE,
    BACKOFF,
    LEFT,
    RIGHT,
    ROTATELEFT,
    ROTATERIGHT,
};

enum MOTORS_SWITCH_STAT
{
    BRL,
    LBR,
    RLB,
};

class Omni3WD
{
public:
    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    Omni3WD(MotorWheel *wheelBack, MotorWheel *wheelRight, MotorWheel *wheelLeft);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned char switchMotorsLeft();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned char switchMotorsRight();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned char switchMotorsReset();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setMotorAll(unsigned int speedMMPS = 0, bool dir = DIR_ADVANCE);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setMotorAllStop();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setMotorAllAdvance(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setMotorAllBackoff(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarStop();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarAdvance(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarBackoff(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarLeft(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarRight(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarRotateLeft(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarRotateRight(unsigned int speedMMPS = 0);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int getCarSpeedMMPS() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarSpeedMMPS(unsigned int speedMMPS = 0, unsigned int ms = 1000);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int setCarSlow2Stop(unsigned int ms = 1000);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int wheelBackSetSpeedMMPS(unsigned int speedMMPS = 0, bool dir = DIR_ADVANCE);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    int wheelBackGetSpeedMMPS() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int wheelRightSetSpeedMMPS(unsigned int speedMMPS = 0, bool dir = DIR_ADVANCE);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    int wheelRightGetSpeedMMPS() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned int wheelLeftSetSpeedMMPS(unsigned int speedMMPS = 0, bool dir = DIR_ADVANCE);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    int wheelLeftGetSpeedMMPS() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    bool PIDEnable(float kc = KC, float taui = TAUI, float taud = TAUD, unsigned int interval = 1000);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    bool PIDRegulate();

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    void delayMS(unsigned int ms = 100, bool debug = false);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    void demoActions(unsigned int speedMMPS = 100, unsigned int duration = 5000,
                     unsigned int uptime = 500, bool debug = false);
    
    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    void demoActions_Orginal(unsigned int speedMMPS = 20, unsigned int ms = 5000, bool debug = false);

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned char getCarStat() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    unsigned char getSwitchMotorsStat() const;

    /**	
     *  @brief brief
     *
     *  @details details
     *
     *  @param param
     *
     *  @returns return
     *
     *  @retval retval
     */
    void debugger(bool wheelBackDebug = true, bool wheelRightDebug = true, bool wheelLeftDebug = true) const;

private:
    MotorWheel *m_wheelBack;
    MotorWheel *m_wheelRight;
    MotorWheel *m_wheelLeft;
    unsigned int (Omni3WD::*carAction)(unsigned int speedMMPS);
    unsigned char m_carStat;
    unsigned char m_switchMotorsStat;
    unsigned char setCarStat(unsigned char stat);
    unsigned char setSwitchMotorsStat(unsigned char switchMotorsStat);

    Omni3WD();
};

#endif
