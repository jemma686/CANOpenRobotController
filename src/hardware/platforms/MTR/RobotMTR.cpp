#include "RobotMTR.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

using namespace Eigen;
using namespace std;


// ═══════════════════════════════════════════════════════════════════════════════
// Construction / destruction
// ═══════════════════════════════════════════════════════════════════════════════

RobotMTR::RobotMTR(const string &robot_name, const string &yaml_config_file)
    : Robot(robot_name, yaml_config_file) {

    // Load YAML overrides before joints are constructed so limits are correct.
    initialiseFromYAML(yaml_config_file);

    // Two Kinco FD123 drives — CAN node IDs 1 (proximal) and 2 (distal).
    addJoint(new JointMT(0,
                         qLimits[0], qLimits[1],          // θ₁ min / max
                         (short int)qSigns[0],
                         -dqMax, dqMax,
                         -tauMax, tauMax,
                         {0,19552,0.0,1.0}, new CopleyDrive(4), "q1"));

    addJoint(new JointMT(1,
                         qLimits[2], qLimits[3],          // θ₂ min / max
                         (short int)qSigns[1],
                         -dqMax, dqMax,
                         -tauMax, tauMax,
                         {0,19552,0.0,1.0}, new CopleyDrive(2), "q2"));

    addInput(keyboard = new Keyboard());

    // KY-040 rotary encoder: CLK=P8_15, DT=P8_17, SW=P8_16
    addInput(encoder = new RotaryEncoder(8, 15, 8, 17, 8, 16));

    // 16×2 I2C LCD: PCF8574 at 0x27 on /dev/i2c-1
    lcd = new LCD1602(0x27, 1);
    lcd->init();

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

RobotMTR::~RobotMTR() {
    for (auto p : joints) delete p;
    joints.clear();
    delete keyboard;
    delete encoder;
    delete lcd;
    inputs.clear();
}


// ═══════════════════════════════════════════════════════════════════════════════
// YAML parameter loading
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::fillParam(YAML::Node node, vector<double> &vec) {
    if (node) {
        for (unsigned int i = 0; i < vec.size(); i++)
            vec[i] = node[i].as<double>();
    }
}

bool RobotMTR::loadParametersFromYAML(YAML::Node params) {
    YAML::Node p = params[robotName];
    if (!p) {
        spdlog::warn("RobotMTR: No parameters for '{}' in YAML — using defaults.",
                     robotName);
        return false;
    }

    // Geometry — loaded from YAML so that tuning never requires a recompile
    if (p["L1"])             L1             = p["L1"].as<double>();
    if (p["L2"])             L2             = p["L2"].as<double>();
    if (p["parallel_ratio"]) parallel_ratio = p["parallel_ratio"].as<double>();

    // Drive envelope (hard-constrained for safety, same as M3)
    if (p["dqMax"])  dqMax  = min(max(0., p["dqMax"].as<double>()), 360.) * M_PI / 180.;
    if (p["tauMax"]) tauMax = min(max(0., p["tauMax"].as<double>()), 80.);


    fillParam(p["qSigns"],       qSigns);
    fillParam(p["frictionVis"],  frictionVis);
    fillParam(p["frictionCoul"], frictionCoul);

    if (p["qLimits"]) {
        for (unsigned int i = 0; i < qLimits.size(); i++)
            qLimits[i] = p["qLimits"][i].as<double>() * M_PI / 180.;
    }

    if (p["qCalibration"]) {
        qCalibration[0] = p["qCalibration"][0].as<double>() * M_PI / 180.;
        qCalibration[1] = p["qCalibration"][1].as<double>() * M_PI / 180.;
        qCalibration[2] = 0.0;
    }

    spdlog::info("RobotMTR: Loaded YAML parameters for '{}'"
                 " (L1={:.3f} m, L2={:.3f} m, parallel_ratio={:.3f}).",
                 robotName, L1, L2, parallel_ratio);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
// CORC initialisation
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotMTR::initialiseJoints() {
    return true;   // joints are created in the constructor
}

bool RobotMTR::initialiseNetwork() {
    spdlog::debug("RobotMTR::initialiseNetwork()");

    for (auto joint : joints) {
        if (!joint->initNetwork()) return false;
    }

    // Allow PDO initialisation to settle
    for (int i = 0; i < 5; i++) { spdlog::debug("."); usleep(10000); }

    for (auto joint : joints) joint->start();

    // Enable drives
    int n = 0;
    for (auto joint : joints) {
        bool ready = false;
        for (int i = 0; i < 10 && !ready; i++) {
            joint->readyToSwitchOn();
            usleep(10000);
            ready = ((joint->getDriveStatus() & 0x01) == 0x01);
        }
#ifndef NOROBOT
        if (!ready) {
            spdlog::error("MTR: Failed to enable joint {} (status: 0x{:02x})",
                          n, joint->getDriveStatus());
            return false;
        }
#endif
        n++;
    }

    updateRobot();
    return true;
}

bool RobotMTR::initialiseInputs() {
    return true;   // encoder is registered via addInput() in constructor
}


// ═══════════════════════════════════════════════════════════════════════════════
// Drive mode initialisation (same pattern as RobotM3)
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotMTR::initTorqueControl() {
    spdlog::debug("RobotMTR: Initialising torque control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter torque control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}

bool RobotMTR::initPositionControl() {
    spdlog::debug("RobotMTR: Initialising position control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_POSITION_CONTROL) != CM_POSITION_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter position control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}

bool RobotMTR::initVelocityControl() {
    spdlog::debug("RobotMTR: Initialising velocity control on all joints.");
    bool ok = true;
    for (auto p : joints) {
        if (((JointMT *)p)->setMode(CM_VELOCITY_CONTROL) != CM_VELOCITY_CONTROL) {
            spdlog::error("RobotMTR: Joint {} failed to enter velocity control.",
                          p->getId());
            ok = false;
        }
        ((JointMT *)p)->readyToSwitchOn();
    }
    usleep(2000);
    for (auto p : joints) ((JointMT *)p)->enable();
    return ok;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Calibration
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::applyCalibration() {
    for (unsigned int i = 0; i < joints.size(); i++)
        ((JointMT *)joints[i])->setPositionOffset(qCalibration[i]);
    calibrated = true;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Kinematics
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Forward kinematics ───────────────────────────────────────────────────────
// x = L1·cos(θ₁) + L2·cos(θ₁ + θ₂·r)
// y = L1·sin(θ₁) + L2·sin(θ₁ + θ₂·r)
// z = 0  (horizontal plane)
VM3 RobotMTR::directKinematic(VM3 q) {
    double t1  = q[0];
    double t12 = q[0] + q[1] * parallel_ratio;
    return VM3(
        L1 * cos(t1) + L2 * cos(t12),
        L1 * sin(t1) + L2 * sin(t12),
        0.0
    );
}

// ─── Jacobian ─────────────────────────────────────────────────────────────────
// 2R planar Jacobian embedded in 3×3.
// Columns correspond to joints 0 and 1 (joint 2 does not exist on hardware).
// Row 2 (z-velocity) is the identity passthrough for the planar constraint.
//
//   det(J) = L1·L2·sin(θ₂·r)
//   Singular when θ₂ = 0° (fully extended) or ±180° (fully folded back).
//   Both are outside the operating range θ₂ ∈ [−160°, −30°], so the
//   Jacobian is always invertible in the workspace.
Matrix3d RobotMTR::J() {
    double t1  = joints[0]->getPosition();
    double t2  = (joints.size() > 1 ? joints[1]->getPosition() : 0.0) * parallel_ratio;
    double t12 = t1 + t2;

    double s1  = sin(t1),  c1  = cos(t1);
    double s12 = sin(t12), c12 = cos(t12);

    Matrix3d Jac = Matrix3d::Zero();

    Jac(0, 0) = -L1 * s1 - L2 * s12;    Jac(0, 1) = -L2 * s12;
    Jac(1, 0) =  L1 * c1 + L2 * c12;    Jac(1, 1) =  L2 * c12;
    Jac(2, 2) = 1.0;   // Z identity — maps planar-constraint F.z → τ[2] (discarded)

    return Jac;
}

// ─── Gravity ──────────────────────────────────────────────────────────────────
// Zero — the arm moves in a horizontal plane; gravity acts normal to the
// workspace.  Inertia / Coriolis compensation is left as a TODO once the
// full dynamic model from System Modelling is complete.
VM3 RobotMTR::calculateGravityTorques() {
    return VM3::Zero();
}


// ═══════════════════════════════════════════════════════════════════════════════
// Robot update (called every control cycle)
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::updateRobot() {
    spdlog::trace("RobotMTR::updateRobot()");
    Robot::updateRobot();   // fills jointPositions_, jointVelocities_, jointTorques_

    bool two = joints.size() > 1;
    VM3 q  (joints[0]->getPosition(), two ? joints[1]->getPosition() : 0.0, 0.0);
    VM3 dq (joints[0]->getVelocity(), two ? joints[1]->getVelocity() : 0.0, 0.0);
    VM3 tau(joints[0]->getTorque(),   two ? joints[1]->getTorque()   : 0.0, 0.0);

    // End-effector position via forward kinematics
    endEffPositions  = directKinematic(q);

    // End-effector velocity: dX = J · dq
    Matrix3d _J      = J();
    endEffVelocities = _J * dq;

    // End-effector force from motor torques: F = (Jᵀ)⁻¹ · τ
    // Only computed when J is not near-singular (det check).
    double det = _J.determinant();   // = L1·L2·sin(θ₂·r) — always non-zero in workspace
    if (abs(det) > 1e-6) {
        Matrix3d _Jtinv = (_J.transpose()).inverse();
        endEffForces      = _Jtinv * tau;
        interactionForces = endEffForces;   // gravity is zero; τ_interaction = τ_motor
    }

    if (safetyCheck() != SUCCESS) disable();

    last_update_time = chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Safety
// ═══════════════════════════════════════════════════════════════════════════════

setMovementReturnCode_t RobotMTR::safetyCheck() {
    if (calibrated) {
        if (getEndEffVelocity().norm() > maxEndEffVel) {
            spdlog::error("MTR: End-effector velocity limit exceeded ({:.2f} m/s)!",
                          getEndEffVelocity().norm());
            return OUTSIDE_LIMITS;
        }
    } else {
        for (unsigned int i = 0; i < joints.size(); i++) {
            if (((JointMT *)joints[i])->safetyCheck() != SUCCESS) {
                spdlog::error("MTR: Joint {} safety triggered!", i);
                return OUTSIDE_LIMITS;
            }
        }
    }
    return SUCCESS;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Joint-space setters
// ═══════════════════════════════════════════════════════════════════════════════

// Only tau[0] and tau[1] are applied — there is no third drive on the MTR.
setMovementReturnCode_t RobotMTR::setJointTorque(VM3 tau) {
    return applyTorque({tau[0], tau[1]});
}

setMovementReturnCode_t RobotMTR::setJointPosition(VM3 q) {
    return applyPosition({q[0], q[1]});
}

setMovementReturnCode_t RobotMTR::setJointVelocity(VM3 dq) {
    return applyVelocity({dq[0], dq[1]});
}


// ═══════════════════════════════════════════════════════════════════════════════
// Task-space force setter
// ═══════════════════════════════════════════════════════════════════════════════

// τ = Jᵀ · F  +  τ_friction
// Gravity is zero (horizontal plane) — no τ_gravity term.
// τ[2] is computed (via the Z identity row in J) but never sent to hardware.
setMovementReturnCode_t
RobotMTR::setEndEffForceWithCompensation(VM3 F, bool friction_comp) {
    if (!calibrated) return NOT_CALIBRATED;

    VM3 tau = J().transpose() * F;   // 3×3 · 3×1 → 3×1; tau[2] discarded below

    if (friction_comp) {
        const double threshold = 0.03;   // dead-band [rad/s]
        for (unsigned int i = 0; i < joints.size(); i++) {
            double dq = ((JointMT *)joints[i])->getVelocity();
            if (abs(dq) > threshold)
                tau(i) += frictionCoul[i] * sign(dq) + frictionVis[i] * dq;
        }
    }

    return setJointTorque(tau);   // only tau[0] and tau[1] reach hardware
}


// ═══════════════════════════════════════════════════════════════════════════════
// Private drive helpers (mirror RobotM3 pattern exactly)
// ═══════════════════════════════════════════════════════════════════════════════

setMovementReturnCode_t RobotMTR::applyTorque(vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setTorque(torques[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in torque control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} torque error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}

setMovementReturnCode_t RobotMTR::applyPosition(vector<double> positions) {
    if (!calibrated) return NOT_CALIBRATED;
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setPosition(positions[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in position control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} position error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}

setMovementReturnCode_t RobotMTR::applyVelocity(vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t ret = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t code = ((JointMT *)p)->setVelocity(velocities[i]);
        if (code == INCORRECT_MODE) {
            spdlog::error("MTR joint {}: not in velocity control.", p->getId());
            ret = INCORRECT_MODE;
        } else if (code != SUCCESS) {
            spdlog::error("MTR joint {} velocity error: {}",
                          p->getId(), setMovementReturnCodeString[code]);
            ret = UNKNOWN_ERROR;
        }
        i++;
    }
    return ret;
}


// ═══════════════════════════════════════════════════════════════════════════════
// State readers
// ═══════════════════════════════════════════════════════════════════════════════

const VX &RobotMTR::getEndEffPosition()   { return endEffPositions;   }
const VX &RobotMTR::getEndEffVelocity()   { return endEffVelocities;  }
const VX &RobotMTR::getEndEffForce()      { return endEffForces;      }
const VX &RobotMTR::getInteractionForce() { return interactionForces; }


// ═══════════════════════════════════════════════════════════════════════════════
// Diagnostics
// ═══════════════════════════════════════════════════════════════════════════════

void RobotMTR::printStatus() {
    cout << fixed << setprecision(3) << showpos;
    cout << "X=["  << getEndEffPosition().transpose() << " ]  ";
    cout << "dX=[" << getEndEffVelocity().transpose() << " ]  ";
    cout << "F=["  << getEndEffForce().transpose()    << " ]" << endl;
    cout << noshowpos;
}

void RobotMTR::printJointStatus() {
    cout << fixed << setprecision(1) << showpos;
    cout << "q=["   << getPosition().transpose() * 180.0 / M_PI << " deg]  ";
    cout << "dq=["  << getVelocity().transpose() * 180.0 / M_PI << " deg/s]  ";
    cout << "tau=[" << getTorque().transpose()                   << " N·m]  {";
    for (auto joint : joints)
        cout << "0x" << hex << ((JointMT *)joint)->getDriveStatus() << "; ";
    cout << "}" << endl;
    cout << noshowpos;
}
