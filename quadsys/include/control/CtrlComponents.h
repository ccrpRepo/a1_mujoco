#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOinterface.h"
#include "interface/CmdPanel.h"
#include "common/robot.h"
#include "gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif // COMPILE_DEBUG

struct CtrlComponents
{
public:
    CtrlComponents(IOinterface *ioInter, Dynamics *dy_) : ioInter(ioInter),dy(dy_)
    {
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents()
    {
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
        delete dy;
    }
    IOinterface *ioInter;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;
    Dynamics *dy;
    double q[12];
    double qd[12];
    double quaxyz[7];
    double v_base[6];

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv()
    {
        ioInter->sendRecv(lowCmd, lowState);
        for (int i(0); i < 12; ++i)
        {
            q[i] = lowState->motorState[i].q;
            qd[i] = lowState->motorState[i].dq;
        }
        for (int i(0); i < 3; ++i)
        {
            quaxyz[i] = lowState->imu.quaternion[i];
            v_base[i] = lowState->imu.gyroscope[i];
        }
        quaxyz[3] = lowState->imu.quaternion[3];
    }
    void runWaveGen()
    {
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance()
    {
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing()
    {
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave()
    {
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj()
    {
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);
    }

    void set_robot_state()
    {
        Vec3 EstPosition = estimator->getPosition();
        Vec3 EstVelocity = estimator->getVelocity();
        Mat3 B2G_RotMat = estimator->_lowState->getRotMat();
        Mat3 G2B_RotMat = B2G_RotMat.transpose();
        quaxyz[4] = EstPosition(0);
        quaxyz[5] = EstPosition(1);
        quaxyz[6] = EstPosition(2);

        // quaxyz[4] = 0;
        // quaxyz[5] = 0;
        // quaxyz[6] = 0;

        EstVelocity = G2B_RotMat * EstVelocity;
        v_base[3] = EstVelocity(0);
        v_base[4] = EstVelocity(1);
        v_base[5] = EstVelocity(2);

        dy->_robot->set_q(q);
        dy->_robot->set_dq(qd);
        dy->_robot->set_quaxyz(quaxyz);
        dy->_robot->set_vbase(v_base);
    }

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif // COMPILE_DEBUG

private:
        WaveStatus _waveStatus = WaveStatus::SWING_ALL;
    
};

#endif // CTRLCOMPONENTS_H