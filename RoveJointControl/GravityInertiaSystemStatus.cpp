#include <GravityInertiaSystemStatus.h>
#include "RoveBoard.h"

GravityInertiaSystemStatus::GravityInertiaSystemStatus(const double j1CenterOfGravity, const double j1Weight, const double j1Length,
                           const double j2CenterOfGravity, const double j2Weight, const double j2Length,
                           const double j3CenterOfGravity, const double j3Weight, const double j3Length,
                           const double j4CenterOfGravity, const double j4Weight, const double j4Length,
                           const double j5CenterOfGravity, const double j5Weight, const double j5Length,
                           const double j6CenterOfGravity, const double j6Weight, const double j6Length) :
                           J1_CENTER_OF_GRAVITY(j1CenterOfGravity), J1_WEIGHT(j1Weight), J1_LENGTH(j1Length),
                           J2_CENTER_OF_GRAVITY(j2CenterOfGravity), J2_WEIGHT(j2Weight), J2_LENGTH(j2Length),
                           J3_CENTER_OF_GRAVITY(j3CenterOfGravity), J3_WEIGHT(j3Weight), J3_LENGTH(j3Length),
                           J4_CENTER_OF_GRAVITY(j4CenterOfGravity), J4_WEIGHT(j4Weight), J4_LENGTH(j4Length),
                           J5_CENTER_OF_GRAVITY(j5CenterOfGravity), J5_WEIGHT(j5Weight), J5_LENGTH(j5Length),
                           J6_CENTER_OF_GRAVITY(j6CenterOfGravity), J6_WEIGHT(j6Weight), J6_LENGTH(j6Length)
{
}

// Empty because there are no pointers.
GravityInertiaSystemStatus::~GravityInertiaSystemStatus()
{
}

void GravityInertiaSystemStatus::update()
{
    // Math here.
}

double GravityInertiaSystemStatus::getGravity(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Gravity;
    case 2:
        return j2Gravity;
    case 3:
        return j3Gravity;
    case 4:
        return j4Gravity;
    case 5:
        return j5Gravity;
    case 6:
        return j6Gravity;
    default:
        return 0.0;
    }
}

double GravityInertiaSystemStatus::getInertia(uint32_t id)
{
    // Return the appropriate value based on the joint.
    // id 1 corresponds to joint 1, id 2 corresponds to joint 2, etc.
    switch (id)
    {
    case 1:
        return j1Inertia;
    case 2:
        return j2Inertia;
    case 3:
        return j3Inertia;
    case 4:
        return j4Inertia;
    case 5:
        return j5Inertia;
    case 6:
        return j6Inertia;
    default:
        return 0.0;
    }
}
