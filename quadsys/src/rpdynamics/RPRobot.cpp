#include "rpdynamics/RPRobot.h"
#include "rpdynamics/Spatial.h"

Robot::Robot(int NB, int NL)
:_NB(NB), _NL(NL)
{
    _base = new Base();

    _body = new Body[_NB]();
    _joint = new Joint[_NB]();
    _parent = new int[_NB]();
    X_dwtree = new Mat6[_NB]();
    X_uptree = new Mat6[_NB]();
    Xj = new Mat6[_NB]();
    Xq = new Mat6[_NB]();
    T_dwtree = new Mat4[_NB]();
    T_uptree = new Mat4[_NB]();
    Tj = new Mat4[_NB]();
    Tq = new Mat4[_NB]();

    _q = new double[_NB]();
    _dq = new double[_NB]();

    _lpjoint = new LoopJoint[_NL]();

    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
        // _q[i] = 0;
        // _dq[i] = 0;
    }

    for (int i = 0; i < NL; i++)
    {
        _lpjoint[i]._suc = -1;
        _lpjoint[i]._pre = -1;
        _lpjoint[i].Xp.setIdentity(6, 6);
        _lpjoint[i].Xs.setIdentity(6, 6);
    }
    _systick = getSystemTime();
}

// bool Robot::_isUpdate()
// {
//     long long sysnow = getSystemTime();
//     if ((sysnow - _systick) < 500)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

Robot::Robot(int NB)
    : _NB(NB)
{
    _NL = 0;
    _base = (Base *)malloc(sizeof(Base));
    _body = (Body *)malloc(sizeof(Body) * _NB);
    _joint = (Joint *)malloc(sizeof(Joint) * _NB);
    _parent = (int *)malloc(sizeof(int) * _NB);
    X_dwtree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    X_uptree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xj = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xq = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    T_dwtree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    T_uptree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tj = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tq = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    _q = (double *)malloc(sizeof(double) * _NB);
    _dq = (double *)malloc(sizeof(double) * _NB);

    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
    }
    _isUpdated = false;
    _systick = getSystemTime();
}

void Robot::Update_Model()
{

    if (!_isUpdated) // !_isUpdated
    {
        // std::cout << "update!!!" << std::endl;
        if (_base->get_BaseType() == BaseType::Floating)
        {
            Mat4 T = Flt_Transform();
            _base->_fltjoint->_T_Base2Wrd = T;
            AdjointT(T, _base->_fltjoint->_X_Base2Wrd);
            Mat3 R = T.block(0, 0, 3, 3);
            Vec3 p = T.block(0, 3, 3, 1);
            R.transposeInPlace();
            p = -R * p;
            T.setIdentity(4, 4);
            T.block(0, 0, 3, 3) = R;
            T.block(0, 3, 3, 1) = p;
            _base->_fltjoint->_T_Wrd2Base = T;
            AdjointT(T, _base->_fltjoint->_X_Wrd2Base);
            // Mat6 X_temp = _base->_fltjoint->_X_Wrd2Base * _base->_fltjoint->_X_Base2Wrd;
            // std::cout << "X_temp: " << std::endl
            //           << X_temp << std::endl;
        }
        else
        {
            _base->_fltjoint->_T_Base2Wrd.setIdentity();
            _base->_fltjoint->_T_Wrd2Base.setIdentity();
            _base->_fltjoint->_X_Base2Wrd.setIdentity();
            _base->_fltjoint->_X_Wrd2Base.setIdentity();
        }

        for (int i = 0; i < _NB; i++)
        {
            if (_joint[i]._jtype == JointType::RZ)
            {
                Xq[i] = Xrotz(_q[i]); // X_down
                Tq[i] = roz(_q[i]);  // T_down
            }
            else if (_joint[i]._jtype == JointType::RX)
            {
                Xq[i] = Xrotx(_q[i]); // X_down
                Tq[i] = rox(_q[i]);  // T_down
            }
            else if (_joint[i]._jtype == JointType::RY)
            {
                Xq[i] = Xroty(_q[i]); // X_down
                Tq[i] = roy(_q[i]);  // T_down
            }

            X_dwtree[i] = Xj[i] * Xq[i];
            T_dwtree[i] = Tj[i] * Tq[i];

            Mat3 R;
            Vec3 xyz;
            R = T_dwtree[i].block(0, 0, 3, 3).transpose();
            xyz = (-R) * T_dwtree[i].block(0, 3, 3, 1);
            Rp2T(R, xyz, T_uptree[i]);
            AdjointT(T_uptree[i], X_uptree[i]);
            // std::cout << i << ": " << std::endl
            //           << X_dwtree[i] << std::endl;
        }
    }
        
    _isUpdated = true;
}

void a1Robot::update_footEnd()
{
    Mat4 qicifoot_temp;
    Mat4 qicimat_temp;
    qicimat_temp = T_dwtree[0] * T_dwtree[1] * T_dwtree[2] * _lpjoint[0].Ts;
    qicifoot_temp.block(0, 0, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[3] * T_dwtree[4] * T_dwtree[5] * _lpjoint[1].Ts;
    qicifoot_temp.block(0, 1, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[6] * T_dwtree[7] * T_dwtree[8] * _lpjoint[2].Ts;
    qicifoot_temp.block(0, 2, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    qicimat_temp = T_dwtree[9] * T_dwtree[10] * T_dwtree[11] * _lpjoint[3].Ts;
    qicifoot_temp.block(0, 3, 4, 1) = qicimat_temp.block(0, 3, 4, 1);

    // qicifoot_temp = _base->_fltjoint->_T_Base2Wrd * qicifoot_temp;
    _endPfoot = qicifoot_temp.block(0, 0, 3, 4);
}

MatX Robot::Cal_Jacobian(int ib, Coordiante frame)
{
    MatX J;
    J.setZero(6, _NB); // initialize Jacobian matrix

    // initial k set
    int *k, *k_temp;
    k_temp = (int *)malloc(sizeof(int) * _NB);
    k = (int *)malloc(sizeof(int) * _NB);
    int j = ib;
    int num = 0;
    while (j >= 0)
    {
        k_temp[num] = j;
        j = _parent[j];
        num++;
    }
    for (int i = num - 1; i >= 0; --i)
    {
        k[num - 1 - i] = k_temp[i];
    }
    // Jacobian of body ib repesent at base frame
    if (frame == Coordiante::BASE) // base numbered -1
    {
        Mat6 X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num; i++)
        {
            X_down = X_down * X_dwtree[k[i]];
            J.block(0, k[i], 6, 1) = X_down * _joint[k[i]]._S_Body;
        }
    }
    // Jacobian of body ib repesent at body ib coordinate
    else if (frame == Coordiante::BODY)
    {
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = _joint[k[num - 1]]._S_Body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * X_uptree[i];
            J.block(0, k[i - 1], 6, 1) = X_up * _joint[k[i - 1]]._S_Body;
        }
    }
    return J;
}

// q(0:3):quaternion
// q(4:6):xyz
Mat4 Robot::Flt_Transform()
{
    Mat4 T;
    Mat3 R;
    Vec3 xyz;
    xyz << _quat_xyz[4], _quat_xyz[5], _quat_xyz[6];
    // std::cout << "xyz: " << xyz.transpose() << std::endl;
    Eigen::Quaterniond qua(_quat_xyz[0],
                           _quat_xyz[1],
                           _quat_xyz[2],
                           _quat_xyz[3]); // w x y z
    qua.normalize();
    R = qua.matrix();
    // std::cout << "R_base: " <<std::endl<< R << std::endl;
    T.setIdentity(4, 4);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = xyz;
    return T;
}

void a1Robot::witre_urdfData()
{
    Mat3 Ic[12];
    Vec3 com[12];
    double mass[12] = {0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226,
                       0.696, 1.013, 0.226};

    com[0]
        << -0.003311,
        -0.000635, 3.1e-05;
    Ic[0] << 0.000469246, 9.409e-06, -3.42e-07,
        9.409e-06, 0.00080749, 4.66e-07,
        -3.42e-07, 4.66e-07, 0.000552929;

    com[1] << -0.003237, 0.022327, -0.027326;
    Ic[1] << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    com[2] << 0.006435, 0.0, -0.107388;
    Ic[2] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[3] << -0.003311, 0.000635, 3.1e-05;
    Ic[3] << 0.000469246, -9.409e-06, -3.42e-07,
        -9.409e-06, 0.00080749, -4.66e-07,
        -3.42e-07, -4.66e-07, 0.000552929;

    com[4] << -0.003237, -0.022327, -0.027326;
    Ic[4] << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    com[5] << 0.006435, 0.0, -0.107388;
    Ic[5] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[6] << 0.003311, -0.000635, 3.1e-05;
    Ic[6] << 0.000469246, -9.409e-06, 3.42e-07,
        -9.409e-06, 0.00080749, 4.66e-07,
        3.42e-07, 4.66e-07, 0.000552929;

    com[7] << -0.003237, 0.022327, -0.027326;
    Ic[7] << 0.005529065, -4.825e-06, 0.000343869,
        -4.825e-06, 0.005139339, -2.2448e-05,
        0.000343869, -2.2448e-05, 0.001367788;

    com[8] << 0.006435, 0.0, -0.107388;
    Ic[8] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    com[9] << 0.003311, 0.000635, 3.1e-05;
    Ic[9] << 0.000469246, 9.409e-06, 3.42e-07,
        9.409e-06, 0.00080749, -4.66e-07,
        3.42e-07, -4.66e-07, 0.000552929;

    com[10] << -0.003237, -0.022327, -0.027326;
    Ic[10] << 0.005529065, 4.825e-06, 0.000343869,
        4.825e-06, 0.005139339, 2.2448e-05,
        0.000343869, 2.2448e-05, 0.001367788;

    com[11] << 0.006435, 0.0, -0.107388;
    Ic[11] << 0.002997972, 0.0, -0.000141163,
        0.0, 0.003014022, 0.0,
        -0.000141163, 0.0, 3.2426e-05;

    for (int i = 0; i < 12;i++)
    {
        _urdf->_urbody[i] = new urbody(com[i], Ic[i],mass[i]);
    }

    Mat3 Ic_base;
    Vec3 com_base;
    double mass_base = 6.0;
    com_base << 0, 0.0041, -0.0005;
    Ic_base << 0.0158533, 0, 0,
        0, 0.0377999, 0,
        0, 0, 0.0456542;
    _urdf->_urbase = new urbody(com_base, Ic_base, mass_base);

    Mat3 rpy[12];
    Vec3 xyz[12];
    JointType jt[12] = {JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY,
                        JointType::RX, JointType::RY, JointType::RY};

    for (int i = 0; i < 12; i++)
    {
        rpy[i].setIdentity(3, 3);
    }
    xyz[0] << 0.1805, -0.047, 0;
    xyz[1] << 0, -0.0838, 0;
    xyz[2] << 0, 0, -0.2;
    xyz[3] << 0.1805, 0.047, 0;
    xyz[4] << 0, 0.0838, 0;
    xyz[5] << 0, 0, -0.2;
    xyz[6] << -0.1805, -0.047, 0;
    xyz[7] << 0, -0.0838, 0;
    xyz[8] << 0, 0, -0.2;
    xyz[9] << -0.1805, 0.047, 0;
    xyz[10] << 0, 0.0838, 0;
    xyz[11] << 0, 0, -0.2;

    for (int i = 0; i < 12; i++)
    {
        _urdf->_urjoint[i] = new urjoint(jt[i], rpy[i], xyz[i]);
    }

    JointType fltjt = JointType::FLOATING;
    _urdf->_urfltjoint = new urjoint(JointType::FLOATING, Mat3::Identity(), Vec3::Zero());
}

void a1Robot::build_a1()
{
    int parentset[12] = {-1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10};
    // number the parent set
    for (int i = 0; i < _NB; i++)
    {
        _parent[i] = parentset[i];
    }

    // Floating base settings
    _base->set_BaseType(BaseType::Floating);
    _base->set_mcI(_urdf->_urbase);
    _base->_fltjoint = new FltJoint(_urdf->_urfltjoint);
    _base->_fltjoint->_T_Base2Wrd.setIdentity();
    _base->_fltjoint->_T_Wrd2Base.setIdentity();
    _base->_fltjoint->_X_Base2Wrd.setIdentity();
    _base->_fltjoint->_X_Wrd2Base.setIdentity();
    _quat_xyz[0] = 1.0f;
    _quat_xyz[1] = 0.0f;
    _quat_xyz[2] = 0.0f;
    _quat_xyz[3] = 0.0f;
    _quat_xyz[4] = 0.0f;
    _quat_xyz[5] = 0.0f;
    _quat_xyz[6] = 0.0f;

    for (int i = 0; i < _NB; i++)
    {
        _body[i].set_mcI(_urdf->_urbody[i]);
        _joint[i].set_type(_urdf->_urjoint[i]->_jt);
        _joint[i].set_rpy_xyz(_urdf->_urjoint[i]->_rpyMat,
                             _urdf->_urjoint[i]->_xyz);
        Rp2T(_joint[i]._rpyMat, _joint[i]._xyz, Tj[i]);
        AdjointT(Tj[i], Xj[i]);
        _q[i] = 0.0;
        Tq[i] = roz(_q[i]);
        T_dwtree[i] = Tj[i] * Tq[i];
    }

    Mat3 rpy_lp;
    Vec3 xyz_lp;
    rpy_lp << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    xyz_lp << 0, 0, -0.2;
    for (int i = 0; i < _NL; i++)
    {
        _lpjoint[i]._pre = WORLD;
        _lpjoint[i].T.setZero(6, 3);
        _lpjoint[i].T << 0, 0, 0,
                            0, 0, 0,
                            0, 0, 0,
                            1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
        _lpjoint[i]._DOF = 3;
        Rp2T(rpy_lp, xyz_lp, _lpjoint[i].Ts);
        AdjointT(_lpjoint[i].Ts, _lpjoint[i].Xs);
        Mat3 R_t = rpy_lp.transpose();
        Vec3 xyz_t = (-R_t) * xyz_lp;
        Rp2T(R_t, xyz_t, _lpjoint[i].Ts_1);
        AdjointT(_lpjoint[i].Ts_1, _lpjoint[i].Xs_1);

        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp);
        Rp2T(Mat3::Identity(), Vec3::Zero(), _lpjoint[i].Tp_1);
        AdjointT(_lpjoint[i].Tp, _lpjoint[i].Xp);
        AdjointT(_lpjoint[i].Tp_1, _lpjoint[i].Xp_1);
    }
    _lpjoint[0]._suc = 2;
    _lpjoint[1]._suc = 5;
    _lpjoint[2]._suc = 8;
    _lpjoint[3]._suc = 11;

    // Update_Model();
    std::cout
        << std::endl
        << "Model building completed! NB = " << _NB << " NL = " << _NL << std::endl;
    std::cout << "The parent set is [ ";
    for (int i = 0; i < _NB; i++)
        std::cout << _parent[i] << " ";
    std::cout << "]" << std::endl;
}