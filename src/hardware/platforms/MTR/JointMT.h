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

    // TO DO: verify the encoder counts
    int encoderCounts = 8192;  //Encoder counts per turn, multiple of 4, potentially 4096
    double reductionRatio = 1.0;  // direct drive — no gearbox attached

    // Torque scaling: 1000 drive units (DS402 0x6071) = Ipeak × Kt N·m = rated torque.
    // MUST VERIFY: read 0x6076 (Motor Rated Torque) from drive flash via candump/cansend or
    //   PCAN-View. It must equal Ipeak × Kt × 1000 mN·m = 684 mN·m. If the drives were
    //   CME2-commissioned for a different robot, re-run CME2 to set 0x6076 = 684 mN·m.
    
    // TO DO: validate Ipeak
    double Ipeak = 2.7;                  //!< ACK-055-06 continuous current [A] where it hits its thermal ceiling, limit to nominal (should be able to go above functionally)
    double motorTorqueConstant = 0.114;  //!< Maxon EC60 torque constant [N·m/A]

    double driveUnitToJointPosition(int driveValue) { return sign * driveValue * (2. * M_PI) / (double)encoderCounts / reductionRatio; };
    int jointPositionToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * (double)encoderCounts * reductionRatio; };
    double driveUnitToJointVelocity(int driveValue) { return sign * driveValue * (2. * M_PI) / 60. / 512. / (double)encoderCounts * 1875 / reductionRatio; };
    int jointVelocityToDriveUnit(double jointValue) { return sign * jointValue / (2. * M_PI) * 60. * 512. * (double)encoderCounts / 1875 * reductionRatio; };
    
    // TO DO: Double check these conversions please!
    double driveUnitToJointTorque(int driveValue) { return sign * driveValue / 1000.0 * motorTorqueConstant * Ipeak * reductionRatio; };
    int jointTorqueToDriveUnit(double jointValue) { return (int)(sign * jointValue / (motorTorqueConstant * Ipeak) * 1000.0 / reductionRatio); };

   public:
    JointMT(int jointID, double q_min, double q_max, short int sign_ = 1, double dq_min = 0, double dq_max = 0, double tau_min = 0, double tau_max = 0, double ipeak = 6.0, double motor_kt = 0.114, Drive *drive = NULL, const std::string& name="");
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
