#include "wbc_controller/wbc_controller.h"

void WBC::dynamics_consistence_task(VecInt4 contact)
{
    int contact_num = 0;
    int row_index = 0;
    MatX K_temp, k_temp;
    // calculate general inertial matrix
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // calculate general bias force
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // calculate constrain matrix K and k
    K_temp = _dy->Cal_K_Flt(k_temp);
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    _K.setZero(contact_num * 3, 18);
    _k.setZero(contact_num * 3, 1);
    for (int i = 0; i < 4;i++)
    {
        if(contact(i) == 1)
        {
            _K.block(3 * row_index, 0, 3, 18) = K_temp.block(3 * i, 0, 3, 18);
            _k.block(3 * row_index, 0, 3, 1) = k_temp.block(3 * i, 0, 3, 1);
            row_index++;
        }
    }
    // calculate nullspace matrix of K
    Eigen::FullPivLU<MatX> lu(_K);
    _G = lu.kernel();

    // std::cout << "_C: " << std::endl
    //           << _C.transpose() << std::endl;
    // dynamics constrain A and b
    MatX A, b;
    int cols, rows;
    rows = _G.rows(); // G : rows x cols   G^T : cols x rows
    cols = _G.cols();
    A.setZero(cols, 30);
    b.setZero(cols, 1);
    _S.setZero(12, 18);
    _S.block(0, 6, 12, 12).setIdentity(12, 12);
    A.block(0, 0, cols, 18) = _G.transpose() * _H;
    A.block(0, 18, cols, 12) = -_G.transpose() * _S.transpose();

    b = -_G.transpose() * _C;

    _eq_task[0] = new eq_Task(A, b, true);
}

void WBC::closure_constrain_task()
{
    MatX A, b;
    int cols, rows;
    rows = _K.rows(); // K : rows x cols 
    cols = _K.cols();
    A.setZero(rows, 30);
    A.block(0, 0, rows, 18) = _K;
    b = _k;

    _eq_task[1] = new eq_Task(A, b, true);
}

void WBC::desired_torso_motion_task(Vec2 ddr_xy)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    A.block(0, 0, 2, 18) = _I_xy;
    b = ddr_xy;

    /*************************************/
    // A.setZero(18, 30);
    // b.setZero(18, 1);
    // A.block(0, 0, 18, 18).setIdentity(18, 18);
    /*************************************/

    _eq_task[2] = new eq_Task(A, b, true);
}

// swing_acc only contain swing foot acceleration, which means 
// the swing cols of swing_acc must set zero when using this function
void WBC::swing_foot_motion_task(Vec34 swing_acc,VecInt4 contact)
{
    MatX A, b;
    Vec3 swing_acc_in_suc;
    int swing_num = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 0)
        {
            swing_num++;
        } 
    }
    if (swing_num == 0)
    {
        _eq_task[3] = new eq_Task(false);
        return;
    }
    A.setZero(swing_num * 3, 30);
    b.setZero(swing_num * 3, 1);
    int row_index = 0;
    _J_swingfoot.setZero(swing_num * 3, 18);
    // _J_swingfoot.block(0, 0, 3, 6) = _dy->_robot->_base->_fltjoint->_X_Base2Wrd.block(3, 0, 3, 6);
    // _J_swingfoot.block(3, 0, 3, 6) = _J_swingfoot.block(0, 0, 3, 6);
    _dy->_robot->update_footEnd();
    
    Mat3 px[4];
    for (int i = 0; i < 4;i++)
    {
        Vec3 footend = _dy->_robot->_endPfoot.block(0,i,3,1);
        px[i] << 0,         -footend(2), footend(1),
                 footend(2), 0,         -footend(0),
                -footend(1), footend(0), 0;
    }

    Eigen::Matrix<double, 3, 6> PIMat;
    Vec6 avp;
    PIMat.block(0, 3, 3, 3).setIdentity(3, 3);
    for (int i = 0; i < 4; i++)
    {
        if (swing_acc.block(0, i, 3, 1).norm() != 0)
        {
            PIMat.block(0, 0, 3, 3) = -px[i];
            _J_swingfoot.block(row_index * 3, 6 + 3 * i, 3, 3) = PIMat * _dy->Cal_Geometric_Jacobain(2+3*i, Coordiante::BASE).block(0, 3 * i, 6, 3);
            avp = _dy->_robot->_base->_fltjoint->_X_Wrd2Base * _dy->_ref_X_s[i] * _dy->_avp[2 + 3 * i];
            b.block(row_index * 3, 0, 3, 1) = swing_acc.block(0, i, 3, 1) - avp.block(3, 0, 3, 1);
            row_index++;
        }
    }
    // std::cout << "R: " << std::endl
    //           << _dy->_robot->_base->_fltjoint->_X_Wrd2Base * _dy->_ref_X_s[0] << std::endl;
    A.block(0, 0, swing_num * 3, 18) = _J_swingfoot;

    _eq_task[3] = new eq_Task(A, b, true);
}

void WBC::body_yaw_height_task(double yaw_acc, double height_acc)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    Vec2 yaw_height;
    yaw_height << yaw_acc, height_acc;
    A.block(0, 0, 2, 18) = _I_yaw_height;
    b = yaw_height;

    _eq_task[4] = new eq_Task(A, b, true);
}

void WBC::body_roll_pitch_task(double roll_acc, double pitch_acc)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    Vec2 roll_pitch;
    roll_pitch << roll_acc, pitch_acc;
    A.block(0,0,2,18) = _I_roll_pitch;
    b = roll_pitch;

    _eq_task[5] = new eq_Task(A, b, true);
}

void WBC::torque_limit_task()
{
    MatX A, b;
    A.setZero(12, 30);
    b.setZero(12, 1);
    A.block(0, 18, 12, 12) = _I_torque;

    _eq_task[6] = new eq_Task(A, b, true);
}

// contact is the contact situation of four foot 1: contact 0: swing
void WBC::friction_cone_task(VecInt4 contact)
{
    MatX D, f;
    int row_index = 0;
    MatX B;
    MatX base_R_s_ext;
    int contact_num = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }

    D.setZero(5 * contact_num, 30);
    f.setZero(5 * contact_num, 1);
    B.setZero(5 * contact_num, 3 * contact_num);
    base_R_s_ext.setIdentity(contact_num * 3, contact_num * 3);
    for (int i = 0; i < contact_num; i++)
    {
        B.block(5 * i, 3 * i, 5, 3) = _Ffri;
    }
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            base_R_s_ext.block(row_index * 3, row_index * 3, 3, 3) = _dy->_robot->_base->_fltjoint->_T_Wrd2Base.block(0, 0, 3, 3) * _dy->_ref_R_s[i];
            row_index++;
        }
    }
    MatX KK_T = _K * _K.transpose();
    MatX K_leftinv = KK_T.inverse() * _K;
    MatX D_temp;
    D_temp.setZero(18, 30);
    D_temp.block(0, 0, 18, 18) = _H;
    D_temp.block(0, 18, 18, 12) = -_S.transpose();
    MatX BRK_inv;
    BRK_inv.setZero(5 * contact_num, 18);
    BRK_inv = B * base_R_s_ext * K_leftinv;
    D = BRK_inv * D_temp;
    f = BRK_inv * _C;
    _D = D;
    _f = f;

    _ineq_task = new ineq_Task(D, f);
}

Vec12 WBC::inverse_dynamics(Vec18 qdd, Vec34 footforce, VecInt4 contact)
{
    /****************************************************************/
    // contact << 1, 1, 1, 1;
    MatX S_T;
    S_T.setZero(18, 12);
    S_T.block(6, 0, 12, 12).setIdentity(12, 12);
    _S.setZero(12, 18);
    _S.block(0, 6, 12, 12).setIdentity(12, 12);
    /****************************************************************/
    Vec12 torque;
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // MatX small_H;
    // small_H = _dy->Cal_Generalize_Inertial_Matrix_CRBA();
    // std::cout << "H: " << std::endl
    //           << small_H << std::endl;
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // MatX smallC = _dy->Cal_Generalize_Bias_force(true);
    // Vec12 errorC = smallC - _C.block(6, 0, 12, 1);
    // double norm_C = errorC.norm();
    // if (norm_C>0.01)
    // std::cout << "C_error: " << errorC.transpose() << std::endl;
    MatX K_temp, k_temp, lenda;
    K_temp = _dy->Cal_K_Flt(k_temp);
    int contact_num = 0;
    int row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    // std::cout << "contact_num: " << contact_num << std::endl;
    _K.setZero(contact_num * 3, 18);
    _k.setZero(contact_num * 3, 1);
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            _K.block(3 * row_index, 0, 3, 18) = K_temp.block(3 * i, 0, 3, 18);
            _k.block(3 * row_index, 0, 3, 1) = k_temp.block(3 * i, 0, 3, 1);
            row_index++;
        }
    }
    MatX A, b;
    A.setZero(18, contact_num * 3 + 12);
    A.block(0, 0, 18, 12) = _S.transpose(); //_S.transpose()
    A.block(0, 12, 18, contact_num * 3) = _K.transpose();
    b = _H * qdd + _C;
    // std::cout << "Hqdd: " << std::endl <<qdd.transpose() * _H.transpose() << std::endl;
    // std::cout << "H: " << std::endl << _H << std::endl;
    MatX D, f;
    MatX bigR, bigF_fri;
    bigR.setZero(contact_num * 3, contact_num * 3);
    bigF_fri.setZero(contact_num * 5, contact_num * 3);
    f.setZero(contact_num * 5, 1);
    row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            bigR.block(row_index * 3, row_index * 3, 3, 3) = _dy->_robot->_base->_fltjoint->_T_Wrd2Base.block(0, 0, 3, 3) * _dy->_ref_R_s[i];
            bigF_fri.block(5 * row_index, 3 * row_index, 5, 3) = _Ffri;
            // f(row_index * 6 + 5) = 80.0;
            row_index++;
        }
    }
    // std::cout << "f: " << f.transpose() << std::endl;
    D.setZero(contact_num * 5, contact_num * 3 + 12);
    D.block(0, 12, contact_num * 5, contact_num * 3) = bigF_fri * bigR;
    // std::cout << "yes" << std::endl;
    solve_QProblem(A, b, D, f);
    // std::cout << "yes" << std::endl;
    Eigen::FullPivLU<MatX> lu(A);
    // std::cout << "A_size: " << A.rows() << " " << A.cols() << std::endl;
    // VecX re1 = lu.solve(b);
    VecX re1 = _di;
    MatX null_A;
    VecX resultX1;
    // std::cout << "yes" << std::endl;
    if (!lu.isInvertible()) // not full rank
    {
        null_A = lu.kernel();
        solve_QProblem(null_A, -re1, D * null_A, D * re1 + f);
        resultX1 = null_A * _di + re1;
        re1 = resultX1;
        VecX frition_cone = D * resultX1 + f;
        // std::cout << " frition_cone " << frition_cone.transpose() << std::endl;
    }
    Vec12 footf;
    footf.setZero();
    for (int i = 0; i < 12; i++)
    {
        torque[i] = re1[i];
    }
    row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if(contact(i) == 1)
        {
            footf[3 * i] = re1[row_index * 3 + 12];
            footf[3 * i + 1] = re1[row_index * 3 + 13];
            footf[3 * i + 2] = re1[row_index * 3 + 14];
            row_index++;
        }
        
    }
    for (int i = 0; i < 4; i++)
    {
        if(contact(i) == 1)
        {
            footf.block(i * 3, 0, 3, 1) = _dy->_ref_R_s[i] * footf.block(i * 3, 0, 3, 1);
        }
    }
    // std::cout << "frition_cone: " << std::endl   
            //   << frition_cone.transpose() << std::endl;
    // std::cout << "frition_cone: " << std::endl
    //           << footf.transpose() * bigF_fri.transpose() << std::endl;
    // MatX error = A * _di - b;
    // Vec12 footuni = vec34ToVec12(footforce);
    // MatX force_temp = _K.transpose() * footf;
    // std::cout << "error: " << error.transpose() << std::endl;
    // std::cout << "KTlda: " << force_temp.transpose() << std::endl;
    // std::cout << "footfmy: " << footf.transpose() << std::endl;
    return torque;
}

void WBC::solve_HOproblem()
{

    MatX C_bar, d_bar, A_bar;
    MatX b_bar;
    C_bar.setIdentity(30, 30);
    d_bar.setZero(30, 1);
    b_bar.setZero(30, 1);
    int maxtask = -1;
    for (int i = 0; i < 7; i++)
    {
        if (_eq_task[i]->_active != true)
        {
            continue;
        }
        A_bar = _eq_task[i]->_A * C_bar;
        b_bar = _eq_task[i]->_b - _eq_task[i]->_A * d_bar;
        solve_QProblem(A_bar, b_bar, _ineq_task->_D * C_bar, _ineq_task->_D * d_bar + _ineq_task->_f); //
        d_bar = d_bar + C_bar * _di;
        Eigen::FullPivLU<MatX> lu(A_bar);
        if (lu.rank() >= A_bar.cols())// if A_bar full rank
        {
            _qdd_torque = d_bar;
            maxtask = i;
            break;
        }
        C_bar = C_bar * lu.kernel();
    }
    _qdd_torque = d_bar;
    // std::cout << "max_task: " << maxtask << std::endl;

    
}

void WBC::solve_QProblem(MatX A, MatX b, MatX D, MatX f)
{
    int n = A.cols();
    int m = 0;
    int p = f.size(); // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    _di.setZero(n, 1);
    _min_ident.setIdentity(n, n);
    _min_ident = _min_ident * 0.0001f;
    _G0 = A.transpose() * A + _min_ident;
    _g0 = A.transpose() * (-b);
    _CI = D;
    _ci0 = f;
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }


    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < p; ++j)
        {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }

    for (int i = 0; i < p; ++i)
    {
        ci0[i] = _ci0(i);
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // std::cout << "min value: " << value << std::endl;

    for (int i = 0; i < n; ++i)
    {
        _di[i] = x[i];
    }
}

VecX WBC::solve_QProblem_Ab(MatX A, MatX b)
{
    VecX result;
    int n = A.cols();
    int m = 0;
    int p = 0; // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    _min_ident.setIdentity(n, n);
    _min_ident = _min_ident * 0.0001f;
    _G0 = A.transpose() * A + _min_ident;
    _g0 = A.transpose() * (-b);
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }


    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // // std::cout << "min value: " << value << std::endl;
    result.setZero(n, 1);
    for (int i = 0; i < n; ++i)
    {
        result(i) = x[i];
    }
    return result;
}
