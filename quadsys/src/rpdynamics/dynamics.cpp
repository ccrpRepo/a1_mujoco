#include "rpdynamics/dynamics.h"

MatX Dynamics::inverse_dynamic_FixedBase(double _ddq[],
                               bool Gra_offset)
{
    MatX dq = Eigen::Map<MatX>(_dq, _NB, 1);
    MatX ddq = Eigen::Map<MatX>(_ddq, _NB, 1);
    MatX torque = Eigen::Map<MatX>(_q, _NB, 1); // torque is the same size of vector(_q)
    Vec6 vJ;            // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    torque.setZero();                                                   
    // Update Robot state, X
    _robot->Update_Model();

    if (Gra_offset)
        _gra << 0, 0, 0, 0, 0, -9.81;
    else
        _gra << 0, 0, 0, 0, 0, 0;

    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < _NB; i++)
    {
        vJ = _joint[i]._S_Body * dq(i);
        if (_parent[i] == -1)
        {
            _v[i] = vJ;
            _a[i] = _X_uptree[i] * (-_gra) + _joint[i]._S_Body * ddq(i);
        }
        else
        {
            _v[i] = _X_uptree[i] * _v[_parent[i]] + vJ;
            _a[i] = _X_uptree[i] * _a[_parent[i]] + _joint[i]._S_Body * ddq(i) + crm(_v[i]) * vJ;
            _avp[i] = crm(_v[i]) * vJ;
        }
        _f[i] = _body[i]._rbi * _a[i] + crf(_v[i]) * _body[i]._rbi * _v[i];
    }

    for (int i = _NB - 1; i >= 0; --i)
    {
        torque(i) = _joint[i]._S_Body.transpose() * _f[i];

        if (_parent[i] != -1)
        {
            _f[_parent[i]] = _f[_parent[i]] + _X_uptree[i].transpose() * _f[i];
        }
    }
    return torque;
}

// Floating base Inverse dynamics using Recursive Newton-Euler Method
// model: The robot model which must been created
// _q_base[]: Floating base orientation and position (4+3 dimension) in inertial coordinate
// _qd_base[]: Floating base twist(6 dimension) in inertial coordinate
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just different by q
// qdd[]: The robot's joint acceleration array, which size is concided with NB,
//        attain from desire joint acceleration
// Gra_offset: if true, then caluate product tau contain the gravity term; if false, vice verse
// a_base: An output vector of this function
MatX Dynamics::inverse_dynamic_Flt(double _ddq[],
                                   bool Gra_offset)
{
    MatX dq = Eigen::Map<MatX>(_dq, _NB, 1);
    MatX ddq = Eigen::Map<MatX>(_ddq, _NB, 1);
    MatX torque = Eigen::Map<MatX>(_q, _NB, 1); // torque is the same size of vector(_q)
    Eigen::Matrix<double, 6, 1> vJ;             // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    torque.setZero();
    Mat6 *IC = new Mat6[_NB];
    // Update Robot state, X
    _robot->Update_Model();

    if (Gra_offset)
        _gra << 0, 0, 0, 0, 0, -9.81;
    else
        _gra << 0, 0, 0, 0, 0, 0;

    Vec6 g = _gra;

    // 在floating base坐标系下表示g
    g = _base->_fltjoint->_X_Wrd2Base * g;
    // 在floating base坐标系下表示v_base
    Vec6 v_base;
    v_base = _base->_fltjoint->_X_Wrd2Base * _robot -> _v_base;

    Mat6 IC_base = _robot->_base->_rbi;
    Vec6 f0 = IC_base * (-g) + crf(v_base) * IC_base * v_base;

    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < _NB; i++)
    {
        vJ = _joint[i]._S_Body * dq(i);
        if (_parent[i] == -1)
        {
            _v[i] = _X_uptree[i] * v_base + vJ;
            _a[i] = _X_uptree[i] * (-g) + _joint[i]._S_Body * ddq(i) + crm(_v[i]) * vJ; //
        }
        else
        {
            _v[i] = _X_uptree[i] * _v[_parent[i]] + vJ;
            _a[i] = _X_uptree[i] * _a[_parent[i]] + _joint[i]._S_Body * ddq(i) + crm(_v[i]) * vJ;
        }
        _f[i] = _body[i]._rbi * _a[i] + crf(_v[i]) * _body[i]._rbi * _v[i];
        IC[i] = _body[i]._rbi;
    }

    for (int i = _NB - 1; i >= 0; --i)
    {
        if (_parent[i] != -1)
        {
            _f[_parent[i]] = _f[_parent[i]] + _X_uptree[i].transpose() * _f[i];
            IC[_parent[i]] += _X_uptree[i].transpose() * IC[i] * _X_uptree[i];
        }
        else // if parent is floating base
        {
            f0 = f0 + _X_uptree[i].transpose() * _f[i];
            IC_base += _X_uptree[i].transpose() * IC[i] * _X_uptree[i];
        }
    }
    _a_base = -(IC_base.inverse()) * f0;
    Vec6 a0_t;
    for (int i = 0; i < _NB; i++)
    {
        if (_parent[i] == -1) // if parent is floating base
        {
            a0_t = _X_uptree[i] * _a_base;
            torque(i) = _joint[i]._S_Body.transpose() * (_f[i] + IC[i] * a0_t);
        }
        else
        {
            a0_t = _X_uptree[i] * a0_t;
            torque(i) = _joint[i]._S_Body.transpose() * (_f[i] + IC[i] * a0_t);
        }
    }
    return torque;
}

// Calculate genralize inertial matrix via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate product tau contain the gravity term; if false, vice verse
MatX Dynamics::Cal_Generalize_Inertial_Matrix_RNEA(MatX Bias_force)
{
    _robot->Update_Model();
    MatX H;
    MatX v;
    MatX a;
    double *qdd_s;
    qdd_s = new double[_NB];
    H.setZero(_NB, _NB); // intialize H
    for (int i = 0; i < _NB; i++)
    {
        // make the select vector qdd_s
        for (int j = 0; j < _NB; j++)
        {
            if (i == j)
                qdd_s[j] = 1;
            else
                qdd_s[j] = 0;
        }
        H.block(0, i, _NB, 1) = inverse_dynamic_FixedBase(qdd_s, true) - Bias_force;
    }
    return H;
}

// Calculate genralize inertial matrix via Composite-Rigid-Body Algorithm
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//        mostly attain from state estimator or just differented by q
MatX Dynamics::Cal_Generalize_Inertial_Matrix_CRBA()
{
    _robot->Update_Model();
    // intialize H
    MatX H;
    H.setZero(_NB, _NB);
    // intialize IC
    Mat6 *IC = new Mat6[_NB];
    
    for (int i = 0; i < _NB; i++)
    {
        IC[i] = _robot->_body[i]._rbi;
    }
    // calculate each body's composite inertial IC
    for (int i = _NB - 1; i >= 0; i--)
    {
        if (_parent[i] != -1)
        {
            IC[_parent[i]] += _X_uptree[i].transpose() * IC[i] * _X_uptree[i];
        }
    }
    // calculate H
    Vec6 fh;
    int j = 0;
    for (int i = 0; i < _NB; i++)
    {
        fh = IC[i] * _joint[i]._S_Body; // in body i coordinate
        H(i, i) = _joint[i]._S_Body.transpose() * fh;
        j = i;
        while (_parent[j] > -1)
        {
            fh = _X_uptree[j].transpose() * fh; // X_up(i) = i_X_lenda(i)
            j = _parent[j];
            H(i, j) = _joint[j]._S_Body.transpose() * fh;
            H(j, i) = H(i, j);
        }
    }
    return H;
}

// Calculate floating base genralize inertial matrix via Composite-Rigid-Body Algorithm
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//        mostly attain from state estimator or just differented by q
MatX Dynamics::Cal_Generalize_Inertial_Matrix_CRBA_Flt(MatX &H_fl)
{
    _robot->Update_Model();
    // intialize H
    Eigen::MatrixXd H;
    H.setZero(_NB, _NB);
    // intialize IC
    Mat6 *IC = new Mat6[_NB];
    Mat6 IC_base;
    IC_base = _robot->_base->_rbi;
    for (int i = 0; i < _NB; i++)
    {
        IC[i] = _robot->_body[i]._rbi;
    }
    // calculate each body's composite inertial IC
    for (int i = _NB - 1; i >= 0; i--)
    {
        if (_parent[i] != -1)
        {
            IC[_parent[i]] += _X_uptree[i].transpose() * IC[i] * _X_uptree[i];
        }
        else // if parent is floating base
        {
            IC_base += _X_uptree[i].transpose() * IC[i] * _X_uptree[i];
        }
    }
    // calculate H
    Vec6 *fh = new Vec6[_NB];
    int j = 0;
    for (int i = 0; i < _NB; i++)
    {
        fh[i] = IC[i] * _joint[i]._S_Body; // in body i coordinate
        H(i, i) = _joint[i]._S_Body.transpose() * fh[i];
        j = i;
        while (_parent[j] > -1)
        {
            fh[i] = _X_uptree[j].transpose() * fh[i]; // X_up(i) = i_X_lenda(i)
            j = _parent[j];
            H(i, j) = _joint[j]._S_Body.transpose() * fh[i];
            H(j, i) = H(i, j);
        }
        fh[i] = _X_uptree[j].transpose() * fh[i];
    }
    // Eigen::MatrixXd F;
    MatX F;
    Mat6 I_flbase;
    F.setZero(6, _NB);
    for (int i = 0; i < _NB; i++)
    {
        F.block(0, i, 6, 1) = fh[i];
    }
    MatX H_Flt;
    H_Flt.setZero(_NB + 6, _NB + 6);
    H_Flt.block(0, 0, 6, 6) = IC_base;
    H_Flt.block(6, 6, _NB, _NB) = H;
    H_Flt.block(0, 6, 6, _NB) = F;
    H_Flt.block(6, 0, _NB, 6) = F.transpose();
    // Eigen::MatrixXd H_fl;
    H_fl = H - F.transpose() * IC_base.inverse() * F;
    I_flbase = IC_base;
    return H_Flt;
}

// Calculate generalize bias force via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate bias force contain the gravity term; if false, vice verse
MatX Dynamics::Cal_Generalize_Bias_force(bool Gra_offset)
{
    MatX Bias_force;
    double *qdd = new double[_NB];
    for (int i = 0; i < _NB; i++)
    {
        qdd[i] = 0;
    }
    Bias_force = inverse_dynamic_FixedBase(qdd, Gra_offset);
    return Bias_force;
}

// Calculate generalize bias force via Recursive Newton-Euler Method
// model: The robot model which must been created
// q_fltbase[]: The robot's floating base orientation and position (dimension :7) representing at body coordinate
// v_fltbase[]: The robot's floating base twist(dimension :6) representing at body coordinate
//   q[]: The robot's joint value array, which size is concide with NB
//  qd[]: The robot's joint velocity array, which size is concided with NB,
//        mostly attain from state estimator or just differented by q
// Gra_offset: if true, then caluate bias force contain the gravity term; if false, vice verse
MatX Dynamics::Cal_Generalize_Bias_force_Flt(bool Gra_offset)
{
    MatX dq = Eigen::Map<MatX>(_dq, _NB, 1);
    MatX torque = Eigen::Map<MatX>(_q, _NB, 1); // torque is the same size of vector(_q)
    Vec6 vJ;             // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    torque.setZero();
    Mat6 *IC = new Mat6[_NB];
    MatX C = Eigen::Map<MatX>(_q, _NB, 1); // torque is the same size of vector(_q)
    // Update Robot state, X
    _robot->Update_Model();

    if (Gra_offset)
        _gra << 0, 0, 0, 0, 0, -9.81;
    else
        _gra << 0, 0, 0, 0, 0, 0;

    Vec6 g = _gra;

    // 在floating base坐标系下表示g
    // g = _base->_fltjoint->_X_Wrd2Base * g;
    // std::cout << "g_base: " << g.transpose() << std::endl;
    // 在floating base坐标系下表示v_base
    Vec6 v_base;
    v_base = _robot->_v_base;
    // std::cout << "v_base: " << v_base.transpose() << std::endl;
    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < _NB; i++)
    {
        vJ = _joint[i]._S_Body * dq(i);
        if (_parent[i] == -1)
        {
            _v[i] = _X_uptree[i] * v_base + vJ;
            _avp[i] = crm(_v[i]) * vJ;
            _a[i] = _X_uptree[i] * (-g) + _avp[i];
        }
        else
        {
            _v[i] = _X_uptree[i] * _v[_parent[i]] + vJ;
            _avp[i] = crm(_v[i]) * vJ;
            _a[i] = _X_uptree[i] * _a[_parent[i]] + _avp[i];
        }
        _f[i] = _robot->_body[i]._rbi * _a[i] + crf(_v[i]) * _robot->_body[i]._rbi * _v[i];
    }
    Vec6 f0 = _robot->_base->_rbi * (-g) + crf(v_base) * _robot->_base->_rbi * v_base;
    for (int i = _NB - 1; i >= 0; --i)
    {
        C(i) = _joint[i]._S_Body.transpose() * _f[i];

        if (_parent[i] != -1)
        {
            _f[_parent[i]] = _f[_parent[i]] + _X_uptree[i].transpose() * _f[i];
        }
        else // if parent is floating base
        {
            f0 = f0 + _X_uptree[i].transpose() * _f[i];
        }
    }

    MatX C_Flt;
    C_Flt.setZero(_NB + 6, 1);
    C_Flt.block(0, 0, 6, 1) = f0;
    C_Flt.block(6, 0, _NB, 1) = C;
    return C_Flt;
}

// Calculate gravity term via Recursive Newton-Euler Method
// model: The robot model which must been created
//   q[]: The robot's joint value array, which size is concide with NB
MatX Dynamics::Cal_Gravity_Term()
{
    _robot->Update_Model();
    MatX Gravity_Term;
    MatX v;
    MatX a;
    double *qdd, *qd;
    qdd = new double[_NB];
    qd = new double[_NB];
    for (int i = 0; i < _NB; i++)
    {
        qdd[i] = 0;
        qd[i] = 0;
    }
    Gravity_Term = inverse_dynamic_FixedBase(qdd, true);
    return Gravity_Term;
}

// Calculate Geometric jacobian matrix, dimensions: 6xNB, which can choose in space form or body form
// model: The robot model which must been created
//         ib: Choose which link the Jacobian revalent to
// Coordinate: Either be INERTIAL_COORDINATE or BODY_COORDINATE
//             if choose INERTIAL_COORDINATE, then will calculte space Jacobian
//             if choose BODY_COORDINATE, then will calculte body Jacobian
MatX Dynamics::Cal_Geometric_Jacobain(int ib, Coordiante coordinate)
{
    _robot->Update_Model();
    MatX J;
    J.setZero(6, _NB); // initialize Jacobian matrix

    // initial k set
    int *k, *k_temp;
    k = new int[_NB];
    k_temp = new int[_NB];
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
    // Jacobian of body ib repesent at inertial frame
    if (coordinate == Coordiante::BASE) // Base numbered -1
    {
        Eigen::Matrix<double, 6, 6> X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num; i++)
        {
            X_down = X_down * _X_dwtree[k[i]];
            J.block(0, k[i], 6, 1) = X_down * _joint[k[i]]._S_Body;
        }
    }
    // Jacobian of body ib repesent at body ib coordinate
    else if (coordinate == Coordiante::BODY)
    {
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = _joint[k[num - 1]]._S_Body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * _X_uptree[i];
            J.block(0, k[i - 1], 6, 1) = X_up * _joint[k[i - 1]]._S_Body;
        }
    }
    else if (coordinate == Coordiante::INERTIAL)
    {
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = _joint[k[num - 1]]._S_Body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * _X_uptree[i];
            J.block(0, k[i - 1], 6, 1) = X_up * _joint[k[i - 1]]._S_Body;
        }
        J = _robot->_base->_fltjoint->_X_Base2Wrd * J;
    }
    return J;
}

// Calculate matrix K which depict the constrian of the model
// model: The robot model which must been created
//     v: each moving link's spatial velocity, size(v) = [6,NB]
//   avp: each moving velocity product acceleration, size(avp) = [6,NB]
//     k: Matrix k is input, and satisfy K*qdd = k
MatX Dynamics::Cal_K_Flt(MatX &k)
{
    _robot->Update_Model();
    int *nc;
    nc = new int[_NL];
    MatX K;
    int col_K = 0, row_K = 0;
    col_K = _NB + 6;
    for (int i = 0; i < _NL; i++)
    {
        nc[i] = _lpjoint[i].T.cols();
    }
    for (int i = 0; i < _NL; i++)
    {
        row_K += nc[i];
    }
    K.setZero(row_K, col_K);
    k.setZero(row_K, 1);

    int row_index = 0;
    for (int nl = 0; nl < _NL; nl++)
    {
        Vec6 vs, vp, as, ap;
        Mat6 lps_X_ref;
        Mat6 lpp_X_ref;
        Mat6 ref_X_p;
        ref_X_p.setIdentity();
        Mat6 ref_X_s;
        ref_X_s.setIdentity();
        Mat6 p_X_ref;
        p_X_ref.setIdentity();
        Mat6 s_X_ref;
        s_X_ref.setIdentity();
        // initial k set
        int *ks_set, *ks_temp;
        ks_set = new int[_NB];
        ks_temp = new int[_NB];
        int j = _lpjoint[nl]._suc;
        int num_ks = 0;
        while (j >= 0)
        {
            ks_temp[num_ks] = j;
            j = _parent[j];
            num_ks++;
        }
        for (int i = num_ks - 1; i >= 0; --i)
        {
            ks_set[num_ks - 1 - i] = ks_temp[i];
        }
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_up = _X_uptree[ks_set[i]] * X_up;
        }
        s_X_ref = X_up * _base->_fltjoint->_X_Wrd2Base;
        lps_X_ref = _lpjoint[nl].Xs_1 * s_X_ref;
        _ref_R_s[nl] = s_X_ref.block(0, 0, 3, 3).transpose();
        //------------------------------------------
        int *kp_set, *kp_temp;
        kp_set = new int[_NB];
        kp_temp = new int[_NB];
        j = _lpjoint[nl]._pre;
        int num_kp = 0;
        while (j >= 0)
        {
            kp_temp[num_kp] = j;
            j = _parent[j];
            num_kp++;
        }
        for (int i = num_kp - 1; i >= 0; --i)
        {
            kp_set[num_kp - 1 - i] = kp_temp[i];
        }
        X_up.setIdentity(6, 6);
        for (int i = 0; i < num_kp; i++)
        {
            X_up = _X_uptree[kp_set[i]] * X_up;
        }
        p_X_ref = X_up * _base->_fltjoint->_X_Wrd2Base;
        lpp_X_ref = _lpjoint[nl].Xp_1 * p_X_ref;

        // --------------------------------------

        // MatX T_ref = lps_X_ref.transpose() * _lpjoint[nl].T;
        MatX T_base = _base->_fltjoint->_X_Base2Wrd.transpose() * lps_X_ref.transpose() * _lpjoint[nl].T;
        // std::cout << nl << ": " << std::endl
        //           << _lpjoint[nl].T.transpose() << std::endl;

        if (nl != 0)
            row_index += nc[nl - 1];

        Mat6 X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num_ks; i++)
        {
            X_down = X_down * _X_dwtree[ks_set[i]];
            K.block(row_index, 6 + ks_set[i], nc[nl], 1) = T_base.transpose() * X_down * _joint[ks_set[i]]._S_Body;
        }
        ref_X_s = _robot->_base->_fltjoint->_X_Base2Wrd * X_down;
        _ref_X_s[nl] = ref_X_s;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num_kp; i++)
        {
            X_down = X_down * _X_dwtree[kp_set[i]];
            K.block(row_index, 6 + kp_set[i], nc[nl], 1) = T_base.transpose() * X_down * _joint[kp_set[i]]._S_Body;
        }
        ref_X_p = _robot->_base->_fltjoint->_X_Base2Wrd * X_down;
        _ref_X_p[nl] = ref_X_p;
        if (_lpjoint[nl]._pre == WORLD)
        {
            vp.setZero(6, 1);
            ap.setZero(6, 1);
        }
        else
        {
            vp = ref_X_p * _v[_lpjoint[nl]._pre];
            ap = ref_X_p * _avp[_lpjoint[nl]._pre];
        }
        if (_lpjoint[nl]._suc == WORLD)
        {
            vs.setZero(6, 1);
            as.setZero(6, 1);
        }
        else
        {
            vs = _base->_fltjoint->_X_Wrd2Base * ref_X_s * _v[_lpjoint[nl]._suc];
            as = _base->_fltjoint->_X_Wrd2Base * ref_X_s * _avp[_lpjoint[nl]._suc];
        }

        K.block(row_index, 0, nc[nl], 6) = T_base.transpose();
        k.block(row_index, 0, nc[nl], 1) = -T_base.transpose() * (as - ap + crm(vs) * vp);
    }

    return K;
}