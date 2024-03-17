#ifndef RPENUM_H
#define RPENUM_H

enum class JointType
{
    FLOATING,
    RX,
    RY,
    RZ,
    // PX,
    // PY,
    // PZ,
    // UXY,
    // UXZ,
    // UYZ,
    // SPHERE
};

enum class Index
{
    BODY,
    WORLD
};

enum BaseType
{
    Fixed,
    Floating
};

enum class Coordiante
{
    BODY,
    BASE,
    INERTIAL
};

#endif