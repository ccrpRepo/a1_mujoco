#ifndef RP_JOINT_H
#define RP_JOINT_H

#include "MathType.h"
#include "RPenum.h"
#include "urdfData.h"

class Joint
{
public:
    Joint() {}
    Joint(urjoint *urj);
    ~Joint() {}
    void set_type(JointType jtype);
    void set_rpy_xyz(Mat3 rpy, Vec3 xyz);

    JointType _jtype;
    Vec6 _S_Body;
    Eigen::Matrix<double, 6, 5> _T_Body;
    double _DOF;
    Mat3 _rpyMat;
    Vec3 _xyz;

private:
};


#endif // RP_JOINT_H