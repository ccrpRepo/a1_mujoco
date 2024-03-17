#ifndef _URDF_DATA_H_
#define _URDF_DATA_H_

#include "MathType.h"
#include "RPenum.h"
#include <iostream>

class urbody
{
public:
    urbody(Vec3 com, Mat3 Ic,double mass)
    {
         write_mcI(com, Ic, mass);
    }
    ~urbody() {}

    void write_mcI(Vec3 com, Mat3 Ic, double mass)
    {
        _com = com;
        _Ic = Ic;
        _mass = mass;
    }
    Vec3 _com;
    Mat3 _Ic;
    double _mass;

private:
};

class urjoint
{
public:
    urjoint(JointType jt, Mat3 rpy, Vec3 xyz)
    {
         write_jtrpyxyz(jt, rpy, xyz);
    }
    ~urjoint() {}
    void write_jtrpyxyz(JointType jt, Mat3 rpy, Vec3 xyz)
    {
        _jt = jt;
        _rpyMat = rpy;
        _xyz = xyz;
    }

    JointType _jt;
    Mat3 _rpyMat;
    Vec3 _xyz;

private:
};

class urdfData
{
    public:
        urdfData(int numbody, int numjoint)
        {
            if(numbody >20 || numjoint > 20)
            {
                std::cout << "the maxinum of body and joint are 20 !" << std::endl;
            }
            else
            {
                _numbody = numbody;
                _numjoint = numbody;
            }
                }
        ~urdfData(){}

        urbody *_urbase;
        urbody *_urbody[20];
        urjoint *_urfltjoint;
        urjoint *_urjoint[20];

    private:
        int _numbody;
        int _numjoint;
};

#endif