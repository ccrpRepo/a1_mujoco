#ifndef IOMUJOCO_H
#define IOMUJOCO_H

#include "interface/IOinterface.h"
#include "string.h"
#include "mjxmacro.h"
#include "mujoco.h"
#include "glfw3.h"

struct _motorcmd
{
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float Kd;
    float Kp;
};

struct _motorstate
{
    uint8_t mode;
    float q;
    float dq;
    float tauEst;
};

struct _imustate
{
    float quaternion[4];
    float gyroscope[3];
    float accelermeter[3];
};

struct mujoco_LowCmd
{
    _motorcmd motorcmd[12];
};

struct mujoco_LowState
{
    _motorstate motorstate[12];
    _imustate imustate;
};

class IOmujoco : public IOinterface
{
public:
    IOmujoco(mjData *d);
    ~IOmujoco();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    mujoco_LowCmd _lowCmd;
    mujoco_LowState _lowState;
    mjData* _d;

    void sendCmd(const LowlevelCmd *lowCmd);
    void recvState(LowlevelState *state);

};

#endif