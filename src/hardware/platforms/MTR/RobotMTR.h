/**
 * \file RobotMTR.h
 * \brief RobotMTR — 2-DOF parallel-linkage tabletop mirror-therapy robot.
 *
 * Kinematic model
 * ───────────────
 * Standard 2D planar robot in the horizontal plane (gravity ⊥ workspace).
 * Parameters loaded from config/MTR_params.yaml:
 *   L1            — proximal link length [m]  (shoulder → elbow pivot)
 *   L2            — distal  link length [m]  (elbow pivot → handle)
 *   parallel_ratio — gear ratio of the parallelogram linkage on joint 2
 *                    (motor encoder counts / actual elbow angle change)
 *
 * Joint convention
 * ────────────────
 *   q[0] = θ₁   proximal joint angle [rad], measured from the +X axis
 *   q[1] = θ₂   distal  joint angle  [rad], measured from link-1 direction
 *
 * Forward kinematics (System Modelling/torquecalc (1).m):
 *   x = L1·cos(θ₁) + L2·cos(θ₁ + θ₂·r)
 *   y = L1·sin(θ₁) + L2·sin(θ₁ + θ₂·r)   where r = parallel_ratio
 *   z = 0  (horizontal plane, mechanically constrained)
 *
 * Jacobian — 2×2 embedded in 3×3 for VM3 / state-machine compatibility
 * ────────────────────────────────────────────────────────────────────
 *   J = │−L1·s₁ − L2·s₁₂   −L2·s₁₂   0│   ← ẋ
 *       │ L1·c₁ + L2·c₁₂    L2·c₁₂   0│   ← ẏ
 *       │              0          0   1│   ← ż  (identity — Z planar-constraint passthrough)
 *
 *   The Z row identity (J(2,2)=1) lets F.z from TherapyControllerState's
 *   planar spring-damper pass through τ[2] without doing harm.
 *   τ[2] is never sent to hardware (only 2 drives exist).
 *   Remove the Z spring-damper in TherapyControllerState::during() when
 *   the physical Z axis is mechanically constrained.
 *
 * Gravity torques
 * ────────────────
 *   Zero — arm moves in a horizontal plane; gravity is normal to the workspace.
 *   Placeholder functions exist for inertia/Coriolis once modelling is complete.
 *
 * Drive hardware
 * ──────────────
 *   2 × Kinco FD123 drives on CAN bus (node IDs 1 and 2), interfaced via
 *   JointM3 objects.  Reduction ratio and encoder counts are configured in
 *   YAML (defaults match the M3: 22:1, 10 000 cpr).
 *
 * Workspace joint limits (System Modelling/ROM (1).m, accelCalc (1).m)
 * ────────────────────────────────────────────────────────────────────
 *   θ₁ ∈ [−30°,  100°]
 *   θ₂ ∈ [−160°, −30°]   (always negative — elbow bends toward the patient)
 *
 * Calibration
 * ────────────
 *   CalibState drives all joints with a positive torque until motion stops.
 *   The robot must reach well-defined mechanical hard stops.
 *   qCalibration holds the KNOWN angles at those stops.
 *   Default assumes positive torque → joint angle increases → stop at θ_max.
 *   *** Verify with the physical hardware before first power-on. ***
 *
 * Migration path  (M3 test hardware → MTR hardware)
 * ─────────────────────────────────────────────────
 *   1. In MTRobotMachine.cpp replace RobotM3 → RobotMTR (include + setRobot).
 *   2. In MTRobotStates.h replace RobotM3State → RobotMTRState (or template
 *      the base on the robot type).
 *   3. Update CalibState / DoNothingState joint-count loops: 3 → joints.size().
 *   4. Remove the Z spring-damper block from TherapyControllerState::during().
 */

#ifndef ROBOTMTR_H
#define ROBOTMTR_H

#include "JointMT.h"        // Copley-drive-backed joint
#include "Keyboard.h"       // Keyboard input (development / fallback)
#include "RotaryEncoder.h"  // KY-040 rotary encoder with push button
#include "LCD1602.h"        // 16×2 LCD via PCF8574 I2C backpack
#include "Robot.h"          // CORC Robot base class
#include "CopleyDrive.h"    // Copley drive interface

typedef Eigen::Vector3d VM3;   //!< 3-vector (shared alias with M3 state machine)
typedef Eigen::VectorXd VX;    //!< Dynamic-size vector (for FLNLHelper / logging)


class RobotMTR : public Robot {
   public:
    RobotMTR(const std::string &robot_name      = "RobotMTR",
             const std::string &yaml_config_file = "");
    ~RobotMTR();

    Keyboard      *keyboard;  //!< Keyboard input (fallback / development)
    RotaryEncoder *encoder;   //!< KY-040 rotary encoder (S1/CLK=P8_11, S2/DT=P8_12, Key/SW=P8_15)
    LCD1602       *lcd;       //!< 16×2 I2C LCD (SDA=P9_20, SCL=P9_19, addr=0x27)

    // ── Drive mode initialisation ─────────────────────────────────────────────
    bool initTorqueControl()   override;
    bool initPositionControl() override;
    bool initVelocityControl() override;

    // ── CORC pure-virtual interface ───────────────────────────────────────────
    bool initialiseJoints()  override;
    bool initialiseNetwork() override;
    bool initialiseInputs()  override;
    void updateRobot()       override;

    // ── Safety ────────────────────────────────────────────────────────────────
    setMovementReturnCode_t safetyCheck();

    // ── Calibration ───────────────────────────────────────────────────────────
    void applyCalibration();
    bool isCalibrated() const { return calibrated; }
    void decalibrate()        { calibrated = false; }

    // ── Kinematics & dynamics ─────────────────────────────────────────────────
    /** 2×2 Jacobian embedded in 3×3 for VM3 compatibility (see file header). */
    Eigen::Matrix3d J();

    /** 2R planar forward kinematics → end-effector position (x, y, 0). */
    VM3 directKinematic(VM3 q);

    /** Returns VM3::Zero() — horizontal plane, gravity is perpendicular. */
    VM3 calculateGravityTorques();

    // ── Joint-space setters ───────────────────────────────────────────────────
    /** Applies tau[0] and tau[1] to the two physical joints; tau[2] ignored. */
    setMovementReturnCode_t setJointTorque(VM3 tau);
    setMovementReturnCode_t setJointPosition(VM3 q);
    setMovementReturnCode_t setJointVelocity(VM3 dq);

    // ── Task-space setter (matches RobotM3 API used by the state machine) ─────
    /** τ = Jᵀ·F + τ_friction  (no gravity term — horizontal plane). */
    setMovementReturnCode_t setEndEffForceWithCompensation(VM3 F,
                                                           bool friction_comp = true);

    // ── State readers (VX for FLNLHelper / CSV logging) ──────────────────────
    const VX &getEndEffPosition();
    const VX &getEndEffVelocity();
    const VX &getEndEffForce();
    const VX &getInteractionForce();

    void printStatus();
    void printJointStatus();

   private:
    // ─────────────────────────────────────────────────────────────────────────
    // Parameters — defaults from system modelling; overridden by YAML if provided
    // ─────────────────────────────────────────────────────────────────────────

    // Geometry (System Modelling/torquecalc (1).m) — MUST match YAML
    double L1             = 0.44;   //!< Proximal link [m]
    double L2             = 0.32;   //!< Distal   link [m]
    double parallel_ratio = 1.0;    //!< Joint-2 parallelogram transmission ratio

    // Drive envelope
    double dqMax  = 360.0 * M_PI / 180.0;   //!< Max joint speed  [rad/s]
    double tauMax =  42.0;                   //!< Max joint torque [N·m]

    // Per-joint drive parameters (index 0 = proximal, index 1 = distal)
    std::vector<double> iPeakDrives  = { 42.0,  42.0};   //!< Peak drive current [A]
    std::vector<double> motorCstt    = {0.132, 0.132};   //!< Torque constant    [N·m/A]
    std::vector<double> qSigns       = {  1.0,   1.0};   //!< Sign correction (CW/CCW)

    // Friction model — tune via identification on real hardware
    std::vector<double> frictionVis  = {0.2, 0.2};   //!< Viscous  [N·m·s/rad]
    std::vector<double> frictionCoul = {0.5, 0.5};   //!< Coulomb  [N·m]

    // Joint limits [rad] layout: {θ1_min, θ1_max, θ2_min, θ2_max}
    // Source: System Modelling/ROM (1).m and accelCalc (1).m
    std::vector<double> qLimits = {
        -30.0 * M_PI / 180.0,    100.0 * M_PI / 180.0,   // θ₁ ∈ [−30°, 100°]
       -160.0 * M_PI / 180.0,    -30.0 * M_PI / 180.0    // θ₂ ∈ [−160°, −30°]
    };

    // NOTE: Step 4 — verify qCalibration against the physical robot before the
    //   first calibrated run. CalibState drives both joints with a small positive
    //   torque until motion stops, then calls applyCalibration() which sets these
    //   angles as the known position at the hard stops. If the values below are
    //   wrong the forward kinematics and all end-effector force control will be
    //   incorrect. Measure the actual stop angles with a protractor or encoder
    //   readout and update here (degrees are converted to radians at construction).
    //   qCalibration[0] = θ₁ at stop,  qCalibration[1] = θ₂ at stop.
    VM3 qCalibration = {
         100.0 * M_PI / 180.0,   // θ₁_max stop — VERIFY on hardware
         -30.0 * M_PI / 180.0,   // θ₂_max stop (least-flexed position) — VERIFY on hardware
          0.0
    };

    bool calibrated = false;

    // ── Safety envelope ───────────────────────────────────────────────────────
    double maxEndEffVel   = 2.0;    //!< Max end-effector speed         [m/s]
    double maxEndEffForce = 20.0;   //!< Max end-effector force (XY)    [N]  — patient contact limit

    // ── Cached end-effector state (VX for FLNLHelper compatibility) ──────────
    VX endEffPositions   = VX::Zero(3);
    VX endEffVelocities  = VX::Zero(3);
    VX endEffForces      = VX::Zero(3);
    VX interactionForces = VX::Zero(3);

    double last_update_time = 0.0;

    // ── YAML parameter loading ────────────────────────────────────────────────
    bool loadParametersFromYAML(YAML::Node params) override;
    void fillParam(YAML::Node node, std::vector<double> &vec);

    // ── Private drive helpers (same pattern as RobotM3) ──────────────────────
    setMovementReturnCode_t applyTorque(std::vector<double> torques);
    setMovementReturnCode_t applyPosition(std::vector<double> positions);
    setMovementReturnCode_t applyVelocity(std::vector<double> velocities);
};

#endif  // ROBOTMTR_H
