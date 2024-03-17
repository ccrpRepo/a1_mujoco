#ifndef _RP_ROBOT_H
#define _RP_ROBOT_H

#include "RPBody.h"
#include "RPJoint.h"
#include "MathType.h"
#include "TimeCounter.h"
#include "urdfData.h"

#define WORLD -2

class LoopJoint : public Joint
{
public:
    LoopJoint() {}
    ~LoopJoint() {}

    Mat6 Xp; // loop joint's frame represent to predessor link's frame
    Mat6 Xs; // loop joint's frame represent to successor link's frame
    Mat4 Tp; // loop joint's frame represent to predessor link's frame
    Mat4 Ts; // loop joint's frame represent to successor link's frame
    MatX T;  // constrian force space represent at loop joint frame

    Mat4 Tp_1; 
    Mat4 Ts_1; 
    Mat6 Xp_1;
    Mat6 Xs_1;

    MatX T_lp;

    int _suc;
    int _pre;

private:
};

class FltJoint : public Joint
{
public:
    FltJoint(urjoint *urj) : Joint(urj)
    {
        _X_Base2Wrd.setIdentity();
        _X_Base2Wrd.setIdentity();
        S_flt.setIdentity();
    }
    ~FltJoint() {}

    Mat6 S_flt;
    Mat6 _X_Wrd2Base;
    Mat6 _X_Base2Wrd;
    Mat4 _T_Wrd2Base;
    Mat4 _T_Base2Wrd;

private:
};

class Base : public Body
{
public:
    Base() {}
    Base(BaseType basetype)
        : _basetype(basetype) {}
    ~Base() {}
    FltJoint *_fltjoint;

    BaseType get_BaseType() { return _basetype; }
    void set_BaseType(BaseType bt) { _basetype = bt; }
    BaseType _basetype;
    private: 
};

class Robot
{
    public:
        Robot(int NB, int NJ);
        Robot(int NB);
        ~Robot() {}

        int _NB;
        int _NL;
        int *_parent;

        Base *_base;
        Body *_body;
        Joint* _joint;
        LoopJoint *_lpjoint;
        double _quat_xyz[7];
        Vec6 _v_base;
        double *_q;
        double *_dq;

        Mat6 *X_dwtree;    // body(i) coordinate respect to body(i-1) coordinate
        Mat6 *X_uptree;    // body(i-1) coordinate respect to body(i) coordinate
        Mat6 *Xj;          // joint(i) coordinate respect to body(i-1) coordinate
        Mat6 *Xq;          // body(i) coordinte respect to joint(i) coordinte

        Mat4 *T_dwtree;    // body(i) coordinate respect to body(i-1) coordinate
        Mat4 *T_uptree;    // body(i-1) coordinate respect to body(i) coordinate
        Mat4 *Tj;          // joint(i) coordinate respect to body(i-1) coordinate
        Mat4 *Tq;          // body(i) coordinte respect to joint(i) coordinte
        long long _systick;
        bool _isUpdated;

        void Update_Model();
        MatX Cal_Jacobian(int ib, Coordiante frame);
        Mat4 Flt_Transform();
        void set_q(double q[])
        {
            for (int i = 0; i < _NB;i++)
            {
                _q[i] = q[i];
                // std::cout << i << ": " << _q[i] <<std::endl;
            }
        }
        void set_dq(double dq[])
        {
            for (int i = 0; i < _NB; i++)
            {
                _dq[i] = dq[i];
                // std::cout << i << ": " << _q[i] <<std::endl;
            }
        }
        void set_quaxyz(double q[])
        {
            for (int i = 0; i < 7; i++)
            {
                _quat_xyz[i] = q[i];
                // std::cout << i << ": " << _q[i] <<std::endl;
            }
        }
        void set_vbase(double v[])
        {
            for (int i = 0; i < 6; i++)
            {
                _v_base(i) = v[i];
                // std::cout << i << ": " << _q[i] <<std::endl;
            }
        }
        // bool _isUpdate();

    private:
};

class a1Robot : public Robot
{
    public:
        a1Robot():Robot(12,4)
        {
            _urdf = new urdfData(12, 12);
            witre_urdfData();
            build_a1();
            _systick = getSystemTime();
            std::cout << "Initialize Robot a1 completed..."
                      << "time stamp: " << _systick << std::endl;
        }
        ~a1Robot()
        {
            delete _urdf;
        }
        urdfData *_urdf;
        Vec34 _endPfoot;

        void update_footEnd();
        void witre_urdfData();
        void build_a1();

    private:
};

#endif