#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
      _est(ctrlComp->estimator), _phase(ctrlComp->phase),
      _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
      _balCtrl(ctrlComp->balCtrl)
{
    _dy = ctrlComp->dy;
    _wbc = new WBC(_dy);
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;
    _tau_last.setZero();

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780;
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif


    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();


    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

State_Trotting::~State_Trotting()
{
    delete _gait;
}

void State_Trotting::enter()
{
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_Trotting::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else
    {
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run()
{
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcQQd();
    calcTau();
    
    if (checkStepOrNot())
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        // _ctrlComp->setAllStance();
        _ctrlComp->setStartWave();
    }
    
    // for (int i = 0; i < 4;i++)
    // {
    //     if((*_contact)(i) == 1)
    //     {
    //         _tau.block(i * 3, 0, 3, 1) = _wbc->_qdd_torque.block(i * 3 + 18, 0, 3, 1);
    //     }
    // }
    _tau = _wbc->_qdd_torque.block(18, 0, 12, 1);
    // std::cout << "tau:" << _tau.transpose() << std::endl;
    // std::cout << "torque:" << _wbc->_qdd_torque.block(18, 0, 12, 1).transpose() << std::endl
    //           << std::endl;
    for (int i = 0; i < 12; i++)
    {
        if (_tau(i) > 33.5 || _tau(i) < -33.5)
        {
            std::cout << "joint(" << i << ") OUT OF RANGE!" << std::endl;
        }
    }
    // std::cout << "tau:" << _tau.transpose() << std::endl;
    _lowCmd->setTau(_tau);
    _tau_last = _tau;

    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
        }
        else
        {
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_Trotting::checkStepOrNot()
{
    if ((fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20))
    {
        return true;
    }
    else
    { 
        return false;
        // return true; //
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz)
{
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0;
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd()
{
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Trotting::calcCmd()
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Trotting::calcTau()
{
    std::cout << std::fixed << std::setprecision(2);
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
            // std::cout << "err   " << _posFeetGlobalGoal.col(i).transpose() << std::endl;
        }
    }
    // std::cout << "_posFeetGlobalGoal: " << _posFeetGlobalGoal << std::endl;
    // std::cout << "_velFeetGlobalGoal: " << _velFeetGlobalGoal << std::endl;
    /**************************************************************************************************************/
    Vec3 dw_base = _G2B_RotMat * _dWbd;
    // dw_world << 0, 0, 1;
    Vec3 ddp_base = _G2B_RotMat * _ddPcd;
    // std::cout << "ddp: " << ddp_base.transpose() << std::endl;
    // std::cout << "dwb: " << dw_base.transpose() << std::endl;
    // dw_base.setZero();
    // ddp_world.setZero();
    _wbc->dynamics_consistence_task(*_contact);
    _wbc->closure_constrain_task();
    Vec2 ddr_xy;
    ddr_xy << ddp_base(0) * 0.3, ddp_base(1) * 1.5;
    // ddr_xy.setZero();
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec34 swingforceFeetBase = _G2B_RotMat * _forceFeetGlobal;
    for (int i = 0; i < 4; i++)
    {
        if ((*_contact)(i) == 1)
        {
            swingforceFeetBase.col(i).setZero();
        }
    }
    _wbc->swing_foot_motion_task(swingforceFeetBase, *_contact);
    double yaw_acc = dw_base(2) * 1.4;      //
    double height_acc = ddp_base(2) * 1.0;  //
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = dw_base(0) * 15.0;    //
    double pitch_acc = dw_base(1) * 30.0;   //
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task();
    _wbc->friction_cone_task(*_contact);
    _wbc->solve_HOproblem();

    // std::cout << "eq1: " << std::endl << _wbc->_eq_task[0]->_A << std::endl;
    // std::cout << "b1: " << std::endl
    //           << _wbc->_eq_task[0]->_b.transpose() << std::endl;

    /*******************************************************************************************************************/
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    // for (int i = 0; i < 4; i++)
    // {
    //     footforce_foot.col(i) = -_wbc->_dy->_ref_R_s[i].transpose() * _forceFeetBody.col(i);
    // }
    // torque_inv = _wbc->inverse_dynamics(qdd, -_forceFeetBody, *_contact);
    // std::cout << "force: " << std::endl
    //           << _forceFeetBody << std::endl;
    _q = vec34ToVec12(_lowState->getQ());
    // std::cout << "force :" << std::endl
    //           << _forceFeetGlobal << std::endl;
    _tau = _robModel->getTau(_q, _forceFeetBody);
    // _tau_wbc = torque_inv;
    // Vec12 footuni = vec34ToVec12(-_forceFeetBody);
    // std::cout << "footuni: " << footuni.transpose() << std::endl;
}

void State_Trotting::calcQQd()
{
    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int i(0); i < 4; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}