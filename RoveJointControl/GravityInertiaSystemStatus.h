/*
 * SystemStatus.h
 *
 * Calculates the Gravity torque and inertia of the arm with values from all joints.
 */

#ifndef ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_
#define ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_

#include "Roveboard.h"
#include "RoveJointControl/AbstractFramework.h"
#include "RoveJointUtilities.h"

//enum representing the different models of arms that are capable of being computed in this class.
typedef enum ArmModel
{
  //the arm for mrdt's 2017 rover called Gryphon
  GryphonArm,
  AtlasArm
} ArmModel;

typedef struct GryphonArmConstants
{
  // Constants.
  const double GRIPPER_WEIGHT;
  const double GRIPPER_LENGTH;
  const double GRIPPER_CENTER_OF_GRAVITY;
  const double FOREARM_WEIGHT;
  const double FOREARM_LENGTH;
  const double FOREARM_CENTER_OF_GRAVITY;
  const double BICEP_WEIGHT;
  const double BICEP_LENGTH;
  const double BICEP_CENTER_OF_GRAVITY;

  FeedbackDevice* const JOINT1ANGLE;
  FeedbackDevice* const JOINT2ANGLE;
  FeedbackDevice* const JOINT3ANGLE;
  FeedbackDevice* const JOINT4ANGLE;
  FeedbackDevice* const JOINT5ANGLE;
  FeedbackDevice* const JOINT6ANGLE;
} GryphonArmConstants;

typedef struct AtlasArmConstants
{
  // Constants.
  const double fcgx;
  const double fcgy;
  const double fcgz;
  const double bcgx;
  const double bcgy;
  const double bcgz;
  const double th1offset;
  const double d1;
  const double a1;
  const double alpha1;
  const double th2offset;
  const double d2;
  const double a2;

  const double alpha2;
  const double th3offset;
  const double d3;
  const double a3;
  const double alpha3;
  const double th4offset;
  const double a4;

  const double alpha4;
  const double th5offset;
  const double d5;
  const double a5;
  const double alpha5;
  const double th6offset;
  const double d6;

  const double alpha6;
  double OpPointoffset[3];

  FeedbackDevice* const JOINT1ANGLE;
  FeedbackDevice* const JOINT2ANGLE;
  FeedbackDevice* const JOINT3ANGLE;
  FeedbackDevice* const JOINT4ANGLE;
  FeedbackDevice* const JOINT5ANGLE;
  FeedbackDevice* const JOINT6ANGLE;

  AtlasArmConstants(double fcgx, double fcgy, double fcgz, double bcgx, double bcgy, double bcgz, double th1offset, double d1, double a1, double alpha1,
                    double th2offset, double d2, double a2, double alpha2, double th3offset, double d3, double a3, double alpha3, double th4offset,
                    double a4, double alpha4, double th5offset, double d5, double a5, double alpha5, double th6offset, double d6, double alpha6,
                    double opPointOffsetx, double opPointOffsety, double opPointOffsetz, FeedbackDevice* joint1Angle, FeedbackDevice* joint2Angle,
                    FeedbackDevice* joint3Angle, FeedbackDevice* joint4Angle, FeedbackDevice* joint5Angle, FeedbackDevice* joint6Angle)
                    : fcgx(fcgx), fcgy(fcgy), fcgz(fcgz), bcgx(bcgx), bcgy(bcgy), bcgz(bcgz), th1offset(th1offset), d1(d1), a1(a1), alpha1(alpha1),
                      th2offset(th2offset), d2(d2), a2(a2), alpha2(alpha2), th3offset(th3offset), d3(d3), a3(a3), alpha3(alpha3), th4offset(th4offset),
                      a4(a4), alpha4(alpha4), th5offset(th5offset), d5(d5), a5(a5), alpha5(alpha5), th6offset(th6offset), d6(d6), alpha6(alpha6),
                      JOINT1ANGLE(joint1Angle), JOINT2ANGLE(joint2Angle), JOINT3ANGLE(joint3Angle), JOINT4ANGLE(joint4Angle), JOINT5ANGLE(joint5Angle), JOINT6ANGLE(joint6Angle)
                    {
                      OpPointoffset[0] = opPointOffsetx, OpPointoffset[1] = opPointOffsety, OpPointoffset[2] = opPointOffsetz;
                    }


} AtlasArmConstants;

//this class is fairly unique, it's designed to be able to compute how much torque the robotic arm is under at any given moment due to gravity
//and inertia.
//It does this by taking in mechanical constants about the arm and tracks the different feedback device's needed to figure out where the arm is
//currently at.
//Note that this class needs its own separate call to compute its math separate from the rest of the RJC; it has an update() function that should
//be called periodically, this will signal to the class that it needs to update its results for how much torque the arm is currently under.
//The rest of the RJC that tries to compensate for gravity or inertia will call this class everytime they are asked to update their movements.
//See the readme.md for more info.
class GravityInertiaSystemStatus
{
  private:

    // Member variables.
    double j1Gravity;
    double j1Inertia;
    double j2Gravity;
    double j2Inertia;
    double j3Gravity;
    double j3Inertia;
    double j4Gravity;
    double j4Inertia;
    double j5Gravity;
    double j5Inertia;
    double j6Gravity;
    double j6Inertia;

    void DHTrans(float th, float d, float a, float alpha, double A1[4][4]);

protected:

    float positionToRad(uint32_t p_units);
    const ArmModel Model;
    const void* ArmModelConstants;

public:
    // Description:     Constructor
    // Arguments:       gripperCenterOfGravity - Center of gravity for gripper. Inches
    //                  gripperWeight          - Weight for gripper. Pounds
    //                  gripperLength          - Length for gripper. Inches
    //                  forearmCenterOfGravity - Center of gravity for forearm.
    //                  forearmWeight          - Weight for forearm.
    //                  forearmLength          - Length for forearm.
    //                  bicepCenterOfGravity   - Center of gravity for bicep.
    //                  bicepWeight            - Weight for bicep.
    //                  bicepLength            - Length for bicep.
    //
    // Derived From:    Nothing
    GravityInertiaSystemStatus(ArmModel model, void* armModelConstants);

    ~GravityInertiaSystemStatus();

    // Description: Calculates the gravity and inertia values of all joints.
    //              Uses the joint constants in the calculations, so no parameters are necessary.
    // Arguments:   None
    // Returns:     Nothing
    void update();

    // Description: Returns the calculated gravity of the desired joint.
    // Arguments:   id - the ID of the joint requesting info.
    // Returns:     the torque gravity of the given joint.
    double getGravity(uint32_t id);

    // Description: Returns the calculated inertia of the desired joint.
    // Arguments:   id - the ID of the joint requesting info.
    // Returns:     the inertia of the given joint.
    double getInertia(uint32_t id);
};

#endif /* ROVEJOINTCONTROL_ARMBOARDSOFTWARE_ROVEJOINTCONTROL_SYSTEMSTATUS_H_ */





