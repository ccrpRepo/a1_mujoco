#ifndef _SPATIAL_H_
#define _SPATIAL_H_

#include <eigen3/Eigen/Dense>
#include <math.h>
#include "MathType.h" 

Mat6 xlt(Vec3 r);
Mat6 Xrotx(double theta);
Mat6 Xroty(double theta);
Mat6 Xrotz(double theta);
Mat6 rotR(Mat3 R);
Mat4 Rp2T(Mat3 R, Vec3 p);
Mat4 rox(double theta);
Mat4 roy(double theta);
Mat4 roz(double theta);
Mat4 Flt_Transform(double q[]);
Mat6 crm(Vec6 v);
Mat6 crf(Vec6 v);
void Rp2T(Mat3 R, Vec3 p, Mat4 &T);
void AdjointT(Mat4 T, Mat6 &X);
#endif