#ifndef RP_BODY_H
#define RP_BODY_H

#include "MathType.h"
#include "urdfData.h"

class Body
{
    public:
        Body()
        {
            _mass = 0;
            _com.setZero();
            _Ic.setZero();
            _rbi.setZero();
        }
        Body(urbody *urb);
        ~Body() {}
        void mcI_to_rbi();

        double get_mass() { return _mass; }
        Vec3 get_com() { return _com; }
        Mat3 get_Ic() { return _Ic; }
        Mat6 get_rbi() { return _rbi; }
        void set_mcI(urbody *urb);

        double _mass;
        Vec3 _com;
        Mat3 _Ic;
        Mat6 _rbi;

    private:

};

#endif // BODY_H