#include "rpdynamics/RPBody.h"

Body::Body(urbody *urb)
    :_mass(urb->_mass), _com(urb->_com), _Ic(urb->_Ic)
{
    mcI_to_rbi();
}

void Body::set_mcI(urbody *urb)
{
    _mass = urb->_mass;
    _com = urb->_com;
    _Ic = urb->_Ic;
    mcI_to_rbi();
}

void Body::mcI_to_rbi()
{
    Mat3 C;
    Mat3 Ident;
    Ident.setIdentity(3, 3);
    C << 0, -_com(2), _com(1),
        _com(2), 0, -_com(0),
        -_com(1), _com(0), 0;

    _rbi.block(0, 0, 3, 3) = _Ic + _mass * C * C.transpose();
    _rbi.block(0, 3, 3, 3) = _mass * C;
    _rbi.block(3, 0, 3, 3) = _mass * C.transpose();
    _rbi.block(3, 3, 3, 3) = _mass * Ident;
}