#ifndef _WBC_CONTROLLER_H_
#define _WBC_CONTROLLER_H_

#include "rpdynamics/dynamics.h"
#include "rpdynamics/MathType.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"

class eq_Task
{
    public:
        eq_Task(MatX A, VecX b, bool active)
        {
            _A = A;
            _b = b;
            _active = active;
        }
        eq_Task(bool active)
        {
            _active = active;
        }
        ~eq_Task(){}
        MatX _A;
        VecX _b;
        bool _active;

    private:
};

class ineq_Task
{
    public:
        ineq_Task(MatX D, VecX f)
        {
            _D = D;
            _f = f;
        }
        ~ineq_Task() {}
        MatX _D;
        VecX _f;

    private:
};

class WBC
{
public:
    WBC(Dynamics *dy) : _dy(dy)
    {
        _I_xy.setZero(2, 18);
        _I_xy << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _I_yaw_height.setZero(2, 18);
        _I_yaw_height << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _I_roll_pitch.setZero(2, 18);
        _I_roll_pitch << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        _I_torque.setIdentity(12, 12);

        _frition = 0.4;
        _Ffri << 1, 0, _frition,
            -1, 0, _frition,
            0, 1, _frition,
            0, -1, _frition,
            0, 0, 1;

        _min_ident.setIdentity(30, 30);
        _min_ident = _min_ident * 0.001;
    }
    ~WBC() 
    {
        delete _dy;
        for (int i = 0; i < 7;i++)
            delete _eq_task[i];
        delete _ineq_task;
        delete _dy;
        delete _dy;
    }

    Dynamics *_dy;
    eq_Task *_eq_task[7];
    ineq_Task *_ineq_task;
    MatX _H, _H_fl, _C, _K, _k, _S, _G, _I_xy, _J_swingfoot, _I_yaw_height, _I_roll_pitch, _I_torque;
    double _frition;
    Eigen::Matrix<double, 5, 3> _Ffri;
    Eigen::Matrix<double, 30, 1> _qdd_torque;
    VecX _di;

    MatX _G0;
    MatX _min_ident;
    VecX _g0;
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;

    MatX _D, _f;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    void dynamics_consistence_task(VecInt4 contact);
    void closure_constrain_task();
    void desired_torso_motion_task(Vec2 ddr_xy);
    void swing_foot_motion_task(Vec34 swing_acc, VecInt4 contact);
    void body_yaw_height_task(double yaw_acc,double height_acc);
    void body_roll_pitch_task(double roll_acc, double pitch_acc);
    void torque_limit_task();
    void friction_cone_task(VecInt4 contact);
    

    Vec12 inverse_dynamics(Vec18 qdd, Vec34 footforce, VecInt4 contact);

    void solve_HOproblem();
    void solve_QProblem(MatX A, MatX b, MatX D, MatX f);
    VecX solve_QProblem_Ab(MatX A, MatX b);

private:
};

#endif //_WBC_CONTROLLER_H_