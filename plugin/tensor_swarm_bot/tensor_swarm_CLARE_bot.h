#ifndef tensorswarm_CLARE_BOT_H
#define tensorswarm_CLARE_BOT_H

// Copyright (C) 2018 deeplearningrobotics.ai
#include <string>

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
/* Definition of the pi-puck distance sensor */
//#include <argos3/plugins/robots/pi-puck/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_lidar_sensor.h>
#include <argos3/core/utility/math/vector3.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include "tensorswarm/AICLAREService.h"
#include "agent_implementation/agent.h"

using namespace argos;

class CTensorSwarmCLAREBot : public CCI_Controller {

public:

    CTensorSwarmCLAREBot();
    virtual ~CTensorSwarmCLAREBot() = default;

    void Init(TConfigurationNode& t_node) override;

    void ControlStep() override;

    void Reset() override;

    /// Returns the scan from the robots laser scanner.
    tensorswarm::LaserScan getLaser() const;

    /// Returns the robots velocity.
    geometry_msgs::Twist getVelocities() const;

    /// Sets the robots velocity.
    void setVelocity(const geometry_msgs::Twist& twist);

    /// Sets a new goal for the robot.
    void setNewGoal(const geometry_msgs::Pose2D& new_goal);

    /// Gets the goal of the robot.
    geometry_msgs::Pose2D getGoal() const;

    /// Returns the distance to the robots goal given its \param currentPosition.
    double goalDistance(const argos::CVector3& currentPosition) const;

    /// Returns the distance to the robots goal given its \param currentPosition.
    double goalDistance(const geometry_msgs::Pose2D &currentPosition) const;

    /**   Provided its \param currentPosition returns the robots progress towards
     *    the goal during the lats timestep. Do not call this function multiple times
     *    in an iteration!
     */
    double goalProgress(const argos::CVector3& currentPosition);

    bool arrived() {return m_arrived;}
    void setArrived() {m_arrived = true;}

    std::shared_ptr<Agent> agentObject;

    double map_width = 10.0;
    double map_height = 10.0;

    Coordinate getActualAgentPosition();
    CRadians getActualAgentOrientation();

private:

    /* Pointer to the differential steering actuator */
    CCI_PiPuckDifferentialDriveActuator* m_pcWheels;
    CCI_SimpleRadiosActuator *m_pcRadiosActuator;
//   CCI_RangeAndBearingActuator* m_pcRangeAndBearingActuator;
    /* Pointer to the pi-puck proximity sensor */
//   CCI_FootBotProximitySensor* m_pcProximity;
    CCI_SimpleRadiosSensor *m_pcRadiosSensor;
    CCI_PiPuckRangefindersSensor* m_pcRangeFindersSensor;
    CCI_PiPuckDifferentialDriveSensor* m_pcDiffDriveSensor;
    CCI_PositioningSensor* m_pcPositioningSensor;
//    CCI_RangeAndBearingSensor* m_pcRangeAndBearingSensor;

    Coordinate previousAgentPosition;
    CRadians previousAgentOrientation;
    int batteryMeasureTicks = 0;
    argos::CVector2 previousMovement = {0, 0};

    double map_width_with_noise_room = 11.0;
    double map_height_with_noise_room = 11.0;

    std::string config_file = "";

    lua_State *L;


    double directions_heatmap[512][512];
    double error_mean_heatmap[512][512];
    double orientation_offset_heatmap[512][512];


    void readHeatmapFromFile(const std::string& filename, double (&heatmap)[512][512]);

    bool mission_start = false;

    static constexpr const Real HALF_BASELINE = 0.07f;
    static constexpr const Real WHEEL_RADIUS = 0.029112741f;

    double current_speed = 0.0;
    double current_angular_speed = 0.0;

    geometry_msgs::Pose2D m_goal;
    double m_previous_goal_distance;
    bool m_goal_progress_called;
    bool m_arrived = false;
};

#endif
