#include "rpdynamics/Spatial.h"

Mat6 xlt(Vec3 r)
{
    Mat6 X;
    X << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, -r[2], r[1], 1, 0, 0,
        r[2], 0, -r[0], 0, 1, 0,
        -r[1], r[0], 0, 0, 0, 1;

    return X;
}

Mat6 Xrotx(double theta)
{
    Mat6 X;

    double c = cos(theta);
    double s = sin(theta);

    X << 1, 0, 0, 0, 0, 0,
        0, c, -s, 0, 0, 0,
        0, s, c, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, c, -s,
        0, 0, 0, 0, s, c;

    return X;
}

Mat6 Xroty(double theta)
{
    Mat6 X;

    double c = cos(theta);
    double s = sin(theta);

    X << c, 0, s, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        -s, 0, c, 0, 0, 0,
        0, 0, 0, c, 0, s,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, -s, 0, c;

    return X;
}

Mat6 Xrotz(double theta)
{
    Mat6 X;

    double c = cos(theta);
    double s = sin(theta);

    X << c, -s, 0, 0, 0, 0,
        s, c, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, c, -s, 0,
        0, 0, 0, s, c, 0,
        0, 0, 0, 0, 0, 1;

    return X;
}

Mat6 rotR(Mat3 R)
{
    Mat6 X;
    X.setZero(6, 6);
    X.block(0, 0, 3, 3) = R;
    X.block(3, 3, 3, 3) = R;

    return X;
}

Mat4 Rp2T(Mat3 R, Vec3 p)
{
    Mat4 T;
    T.setZero(4, 4);
    T(3, 3) = 1;
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = p;
    return T;
}

Mat4 rox(double theta)
{
    Mat4 R;

    double c = cos(theta);
    double s = sin(theta);

    R << 1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1;

    return R;
}

Mat4 roy(double theta)
{
    Mat4 R;

    double c = cos(theta);
    double s = sin(theta);

    R << c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1;

    return R;
}

Mat4 roz(double theta)
{
    Mat4 R;

    double c = cos(theta);
    double s = sin(theta);

    R << c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return R;
}

Mat6 crm(Vec6 v)
{
    Mat6 vcross;
    vcross << 0, -v(2), v(1), 0, 0, 0,
        v(2), 0, -v(0), 0, 0, 0,
        -v(1), v(0), 0, 0, 0, 0,
        0, -v(5), v(4), 0, -v(2), v(1),
        v(5), 0, -v(3), v(2), 0, -v(0),
        -v(4), v(3), 0, -v(1), v(0), 0;

    return vcross;
}

Mat6 crf(Vec6 v)
{
    Mat6 vcross;
    vcross = -crm(v).transpose();

    return vcross;
}

// Ajoint operation: calculate generalize transformation from rotation matrix and distance vector
void AdjointT(Mat4 T, Mat6 &X)
{
    Mat3 R = T.block(0, 0, 3, 3);
    Vec3 p = T.block(0, 3, 3, 1);
    Mat3 pR;
    Mat3 px;
    px << 0, -p(2), p(1),
        p(2), 0, -p(0),
        -p(1), p(0), 0;
    pR = px * R;
    X.setZero(6, 6);
    X.block(0, 0, 3, 3) = R;
    X.block(3, 3, 3, 3) = R;
    X.block(3, 0, 3, 3) = pR;
}

void Rp2T(Mat3 R, Vec3 p, Mat4 &T)
{
    T.setIdentity();
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = p;
}