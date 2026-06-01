/**
 * @brief An implementation of the Drive Object, specifically for the Copley Drive
 *
 */
#include "CopleyDrive.h"

#include <iostream>
#include <set>

CopleyDrive::CopleyDrive(int NodeID) : Drive::Drive(NodeID) {
    OD_Addresses[DIGITAL_IN] = {0x219A, 0x00};
    OD_Addresses[DIGITAL_OUT] = {0X2194, 0x00};
    RPDO_MappedObjects[1] = {CONTROL_WORD};   // no DIGITAL_OUT mapping on Copley
    TPDO_MappedObjects[4] = {};               // 0x219A is not PDO-mappable on Copley drives

    // TPDO3: Torque Actual Value (0x6077, Rated Torque/1000).
    // Requires Motor Rated Torque (0x6076) to be stored in drive flash from CME2 commissioning.
    // If 0x6077 always reads 0, swap to {0x6074, 0x00} (Torque Demand — works without CME2).
    OD_Addresses[ACTUAL_TOR] = {0x6077, 0x00}; // 0x6077 = Torque Actual Value, 0x6074 = Torque Demand
    TPDO_MappedObjects[3] = {ACTUAL_TOR};   // ACTUAL_TOR = 0x6077
}
CopleyDrive::~CopleyDrive() {
    spdlog::debug("CopleyDrive Deleted");
}

bool CopleyDrive::init() {
    spdlog::debug("NodeID {} CopleyDrive::init()", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetErrors();
    if(initPDOs()) {
        resetErrors();
        return true;
    }
    return false;
}

bool CopleyDrive::initPosControl(motorProfile posControlMotorProfile) {
    spdlog::debug("Node     ID {} Initialising Position Control", NodeID);

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */
    return true;
}

bool CopleyDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool CopleyDrive::initVelControl() {
    spdlog::debug("NodeID {} Initialising Velocity Control (default profile)", NodeID);
    motorProfile p;
    p.profileVelocity     = 500000;   // max profile velocity — tune as needed
    p.profileAcceleration = 100000;   // ramp up in ~1 second at target speed
    p.profileDeceleration = 100000;
    sendSDOMessages(Drive::generateVelControlConfigSDO(p));
    return true;
}

bool CopleyDrive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);

    std::vector<std::string> CANCommands;
    std::stringstream sstream;

    // Run wake-and-wiggle (mode 5) once per drive per program run to establish commutation.
    // A static set tracks which node IDs have already been phased so re-entering states
    // does not re-trigger the wiggle (which would fault the drive on the second call).
    // 0x21C2 = 30 → 0.30 A phasing current (units: 0.01 A).  Low enough to align the rotor
    // gently without spinning it far enough to trip the drive's velocity protection.
    // After the first run the result is saved to flash; power-cycling the drive is not needed.
    // Mode 0: use the phase angle stored in drive flash.
    // A mode-5 wake-and-wiggle was run on 2026-06-01 to establish and save this angle.
    // Do not change back to mode 5 without restraining the arm first — the wiggle sweeps
    // ~100 degrees of arm travel which is unsafe near a patient.
    sstream << "[1] " << NodeID << " write 0x21C0 0 i16 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // Standard torque mode config (sets 0x6060 = 4)
    auto torqueSDOs = generateTorqueControlConfigSDO();
    CANCommands.insert(CANCommands.end(), torqueSDOs.begin(), torqueSDOs.end());

    sendSDOMessages(CANCommands);
    return true;
}

std::vector<std::string> CopleyDrive::generatePosControlConfigSDO(motorProfile positionProfile) {
    return Drive::generatePosControlConfigSDO(positionProfile); /*<!execute base class function*/
};

std::vector<std::string> CopleyDrive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    return Drive::generateVelControlConfigSDO(velocityProfile); /*<!execute base class function*/
};

std::vector<std::string> CopleyDrive::generateTorqueControlConfigSDO() {
    return Drive::generateTorqueControlConfigSDO(); /*<!execute base class function*/
}

std::vector<std::string> CopleyDrive::generatePositionOffsetSDO(int offset) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // set mode of operation
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 6";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set the home offset
    sstream << "[1] " << NodeID << " write 0x607C 0 i32 "<< std::dec << offset;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set homing method to 0
    sstream << "[1] " << NodeID << " write 0X6098 0 i8 0";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set control word to start homing
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    // set control word to start homing
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x1f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    return CANCommands;
}

bool CopleyDrive::setPositionOffset(int offset) {
    spdlog::debug("NodeID {} Setting Position Offset", NodeID);

    sendSDOMessages(generatePositionOffsetSDO(offset));

    return true;


}

bool CopleyDrive::setTrackingWindow(INTEGER32 window) {
    spdlog::debug("NodeID {} Tracking Window", NodeID);

    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " write 0x2120 0 i32 " << std::dec << window;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);

    return true;
}

bool CopleyDrive::setFaultMask(UNSIGNED32 mask) {
    spdlog::debug("NodeID {} Fault mask set to {0:x}", NodeID, mask);

    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    sstream << "[1] " << NodeID << " write 0x2182 0 i32 " << std::dec << mask;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);

    return true;
}
