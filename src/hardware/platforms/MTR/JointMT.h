/**
 * \file JointMT.h
 * \author Vincent Crocher
 * \brief An M3 actuated joint
 * \version 0.2
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef JOINTM3_H_INCLUDED
#define JOINTM3_H_INCLUDED

#include <iostream>
#include <cmath>

#include "Joint.h"
#include "CopleyDrive.h"

#define MOTOR_RATED_TORQUE 0.319
#define REDUCTION_RATIO 15

typedef struct JointDrivePairs {
    int drivePosA;
    int drivePosB;
    double jointPosA;
    double jointPosB;
} JointDrivePairs;

/**
 * \brief MTR actuated joints, using Copley drives.
 *
 */
class JointMT : public Joint {
   private:
    const short int sign;
    const double qMin, qMax, dqMin, dqMax, tauMin, tauMax;
    int encoderCounts = 10000;  //Encoder counts per turn
    double JDSlope;
    double JDIntercept;

    double Ipeak;               //!< Drive max current (used in troque conversion)
    double motorTorqueConstant; //!< Motor torque constant

    double driveUnitToJointPosition(int driveValue) { return (driveValue - JDIntercept) / JDSlope; };
    int jointPositionToDriveUnit(double jointValue) { return JDSlope * jointValue + JDIntercept; };
    double driveUnitToJointVelocity(int driveValue) { return ((driveValue) / (JDSlope * 10)); };
    int jointVelocityToDriveUnit(double jointValue) { return (JDSlope * jointValue) * 10; };
    double driveUnitToJointTorque(int driveValue) { int s = (JDSlope > 0) ? 1 : -1; return s * driveValue * (MOTOR_RATED_TORQUE * REDUCTION_RATIO / 1000.0); };
    int jointTorqueToDriveUnit(double jointValue) { int s = (JDSlope > 0) ? 1 : -1; return (int)(s * jointValue / (MOTOR_RATED_TORQUE * REDUCTION_RATIO / 1000.0)); };

    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
 
/*
This is Calibration below, need to adjust during calibration (jointangle in Radians)
 double JointDrivePairs jdp = {0, 1, 0.0, 1.0} = {encoder count at posA, encoder count at posB, joint angle at posA, joint angle at posB}
*/
   public:
    JointMT(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, JointDrivePairs jdp = {0, 1, 0.0, 1.0}, Drive *drive = NULL, const std::string& name="");
    ~JointMT();
    /**
     * \brief Check if current velocity and torque are within limits.
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();

    setMovementReturnCode_t setPosition(double qd);
    setMovementReturnCode_t setVelocity(double dqd);
    setMovementReturnCode_t setTorque(double taud);

    bool initNetwork();
};

#endif
