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
#define REDUCTION_RATIO 1   // direct drive — no gearbox attached

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
    int encoderCounts = 8192;  //Encoder counts per turn, multiple of 4
    double reductionRatio = 1.0;  // direct drive — no gearbox attached

    // NOTE: find in documentation of drive, or shaft on the motor with scale to get force and get 3 points for linear relation
    // Can replace  Ipeak / 1.414 * motorTorqueConstant with just one JOINTCONVERSIONCONST
    double Ipeak = 6.0;               //!< Drive max current (used in torque conversion), A
    double motorTorqueConstant = 0.114; //!< Motor torque constant

    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue * (2. * M_PI) / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio; };
    // DS402: 0x6071 Target Torque is in 0.1% of rated torque; 1000 = Ipeak * Kt
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / 1000.0 * motorTorqueConstant * Ipeak * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return (int)(sign * jointValue / (motorTorqueConstant * Ipeak) * 1000.0 / reductionRatio); };

    /**
     * \brief motor drive position control profile paramaters, user defined.
     *
     */
 
/*
This is Calibration below, need to adjust during calibration (jointangle in Radians)
 double JointDrivePairs jdp = {0, 1, 0.0, 1.0} = {encoder count at posA, encoder count at posB, joint angle at posA, joint angle at posB}
*/
   public:
    JointMT(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, double ipeak = 42.0, double motor_kt = 0.132, Drive *drive = NULL, const std::string& name="");
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
