// Copyright (C) 2018 deeplearningrobotics.ai
#include "tensor_swarm_CLARE_bot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>

#include "ros_helpers.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace tensorswarm;

CTensorSwarmCLAREBot::CTensorSwarmCLAREBot() :
        m_pcWheels(nullptr),
        m_pcRangeFindersSensor(nullptr),
        m_pcRadiosActuator(nullptr),
        m_pcRadiosSensor(nullptr),
        agentObject(nullptr)
{}

void CTensorSwarmCLAREBot::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
    m_pcRadiosActuator = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
    m_pcRadiosSensor = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
    m_pcRangeFindersSensor = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
    m_pcPositioningSensor = GetSensor<CCI_PositioningSensor>("positioning");//    m_pcWheelsSensor = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
//    m_pcLaser = GetSensor<CCI_FootBotLidarSensor>("footbot_lidar");
    GetNodeAttributeOrDefault(t_node, "map_width", map_width, map_width);
    GetNodeAttributeOrDefault(t_node, "map_height", map_height, map_height);
    map_height_with_noise_room = map_height + 1.0; //Room for noise
    map_width_with_noise_room = map_width + 1.0; //Room for noise
    GetNodeAttributeOrDefault(t_node, "config", config_file, config_file);


    agentObject = std::make_shared<Agent>(Agent(this->m_strId, std::max(map_width_with_noise_room, map_height_with_noise_room), config_file));
    agentObject->setWifi(Radio(m_pcRadiosActuator, m_pcRadiosSensor));

    agentObject->differential_drive.setActuator(m_pcWheels);

    readHeatmapFromFile("src/tensorflow/plugin/Thesis_ARGoS/implementation_and_examples/controllers/pipuck_hugo/position_direction_offset.txt", this->directions_heatmap);
    readHeatmapFromFile("src/tensorflow/plugin/Thesis_ARGoS/implementation_and_examples/controllers/pipuck_hugo/orientation_offset.txt", this->orientation_offset_heatmap);
    readHeatmapFromFile("src/tensorflow/plugin/Thesis_ARGoS/implementation_and_examples/controllers/pipuck_hugo/error_heatmap.txt", this->error_mean_heatmap);

    previousAgentPosition = getActualAgentPosition();
    previousAgentOrientation = getActualAgentOrientation();


}

void CTensorSwarmCLAREBot::readHeatmapFromFile(const std::string& filename, double (&heatmap)[512][512]) {
    std::ifstream file(filename);
    std::string line;
    int row_index = 0;
    assert(file.good());
    while (std::getline(file, line) && row_index < 512) {
        double value;
        int col_index = 0;
        //Remove the first character, which is a '{'
        line = line.substr(1);
        //Remove the last two characters, which are a '}' and a ','
        line = line.substr(0, line.size() - 2);
        std::stringstream ss(line);
        while (ss >> value && col_index < 512) {
            heatmap[row_index][col_index] = value;
            if (ss.peek() == ',') ss.ignore();
            col_index++;
        }

        row_index++;
    }
}

int mirrored_mod(int x, int m) {
    int q = std::floor(x / m);
    if (q % 2 == 0) {
        return x % m;
    } else {
        return m - (x % m);
    }
}

void CTensorSwarmCLAREBot::ControlStep() {
    m_goal_progress_called = false;

    static constexpr size_t num_sensors = 4; // Define num_sensors as a static constexpr
    Real proxReadings[num_sensors] = {};
    std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> visitFn =
            [&proxReadings](const CCI_PiPuckRangefindersSensor::SInterface &sensor) {
                proxReadings[sensor.Label] = sensor.Proximity;
            };
    m_pcRangeFindersSensor->Visit(visitFn);

    int agentIdVal = 0;
    for (char c : agentObject->id) {
        agentIdVal += static_cast<int>(c);
    }
    agentIdVal *= 1234; //Multiply with some number to offset the differences between agents.

    for(int i = 0; i < num_sensors; i++){
        auto sensorReading = proxReadings[i];
        double sensorNoiseM = 0.0;
        double sensorNoiseRange = agentObject->config.DISTANCE_SENSOR_NOISE_CM * 100;
        // Add noise to the sensor reading, if it is not the maximum range (nothing hit)
        if(sensorReading != agentObject->config.DISTANCE_SENSOR_PROXIMITY_RANGE) sensorNoiseM = (sensorNoiseRange - (rand() % 2 * sensorNoiseRange)) * 0.0001 ; // Random number between -1 and 1 cm (= 0.01 m), to simulate sensor noise
        agentObject->setLastRangeReadings(i, sensorReading + sensorNoiseM);
    }
//
//
    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    const auto position = positionSensorReading.Position;

    // Simulate sensor readings with noise


    //Orientation noise
    //Agent-dependent noise
    double agentPersonalOrientationNoise = agentObject->config.ORIENTATION_JITTER_DEGREES != 0 ? mirrored_mod((agentIdVal + int((position.GetX() + position.GetY())*100)), int(agentObject->config.ORIENTATION_JITTER_DEGREES*2)) - agentObject->config.ORIENTATION_JITTER_DEGREES : 0;
//    argos::LOG << "Agent personal orientation noise: " << agentPersonalOrientationNoise << std::endl;
    argos::CRadians agentPersonalOrientationNoiseRad = ToRadians(CDegrees(agentPersonalOrientationNoise));

    // Orientation jitter
    double orientationJitterRange = agentObject->config.ORIENTATION_JITTER_DEGREES;
    argos::CRadians orientationJitter = ToRadians(CDegrees((orientationJitterRange - ((double(rand() % 200) / 100.0) * orientationJitterRange)))); ; // Random number between -orientationJitterRange and orientationJitterRange rad, to simulate sensor noise

    //Position noise
    //Agent-dependent noise
    double agentPersonalPositionNoise =  agentObject->config.POSITION_JITTER_CM != 0 ? (mirrored_mod((agentIdVal + int((position.GetX() - position.GetY())*100)), int(agentObject->config.POSITION_JITTER_CM*2)) - agentObject->config.POSITION_JITTER_CM) / 100.0 : 0;
//    argos::LOG << "Agent personal position noise: " << agentPersonalPositionNoise << std::endl;
    //Position jitter
    double positionJitterRange = agentObject->config.POSITION_JITTER_CM / 100.0;
    double positionJitter = (positionJitterRange - ((double(rand() % 200) / 100.0) * positionJitterRange)); // Random number between -positionJitterRange and positionJitterRange m, to simulate sensor noise

    //Get values from heatmap, and convert them into the correct range
    int heatmap_size = sizeof(this->directions_heatmap)/sizeof(this->directions_heatmap[0]);
    int heatmapX = std::floor((-position.GetY() + this->map_width_with_noise_room/2)/(this->map_width_with_noise_room/heatmap_size));
    int heatmapY = std::floor((position.GetX() + this->map_height_with_noise_room/2)/(this->map_height_with_noise_room/heatmap_size));

    //Simulate position estimation offset
    //Direction, including jitter and agent-dependent noise
    double direction_heatmap_value = this->directions_heatmap[heatmapY][heatmapX];
    CRadians error_direction_offset = CRadians(direction_heatmap_value) + orientationJitter + agentPersonalOrientationNoiseRad;
    //Magnitude, including jitter and agent-dependent noise
    double error_mean_heatmap_value = this->error_mean_heatmap[heatmapY][heatmapX] * agentObject->config.POSITION_NOISE_CM / 100.0;
    double error_magnitude = error_mean_heatmap_value + positionJitter + agentPersonalPositionNoise;
    CVector2 position_error_vector = CVector2(error_magnitude, 0).Rotate(error_direction_offset);
    agentObject->setPosition(-position.GetY() + position_error_vector.GetX(), position.GetX() + position_error_vector.GetY()); // X and Y are swapped in the positioning sensor, and we want left to be negative and right to be positive

    //Simulate orientation (IMU) estimation offset
    double orientation_offset_heatmap_value = this->orientation_offset_heatmap[heatmapY][heatmapX]*agentObject->config.ORIENTATION_NOISE_DEGREES;

    CRadians zAngle, yAngle, xAngle;
    const auto orientation = positionSensorReading.Orientation;
    orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    CRadians orientationOffset = ToRadians(CDegrees(orientation_offset_heatmap_value));
    agentObject->setHeading(zAngle + orientationOffset + orientationJitter + agentPersonalOrientationNoiseRad);

//
////#ifdef BATTERY_MANAGEMENT_ENABLED
//    //Update agent battery level
////    if (batteryMeasureTicks % 1 == 0) {
//
    //Get relative vector
    float traveledPathLength = sqrt(pow(this->getActualAgentPosition().x - previousAgentPosition.x, 2) +
                                    pow(this->getActualAgentPosition().y - previousAgentPosition.y, 2));
    CRadians traveledAngle = zAngle - previousAgentOrientation;
    CVector2 traveledVector = CVector2(traveledPathLength, 0).Rotate(traveledAngle);

    this->current_speed = traveledPathLength/agentObject->ticks_per_second;
    this->current_angular_speed = traveledAngle.GetValue()/agentObject->ticks_per_second;
//
//
    auto [usedPower, duration] = agentObject->batteryManager.calculateTotalPowerUsageFromMovement(agentObject.get(), previousMovement, traveledVector);
    agentObject->batteryManager.battery.charge -= usedPower;
//        argos::LOG << "Used power: " << usedPower << "mAh" << std::endl;

    previousAgentPosition = getActualAgentPosition();
    previousAgentOrientation = zAngle;
//        batteryMeasureTicks = 0;
//    }
//    batteryMeasureTicks++;
//#endif

    if (!this->mission_start){
        agentObject->startMission();
        this->mission_start = true;
    }
//    agentObject->doStep();
}

Coordinate CTensorSwarmCLAREBot::getActualAgentPosition() {
    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    const auto position = positionSensorReading.Position;
    return Coordinate{-position.GetY(), position.GetX()}; // X and Y are swapped in the positioning sensor, and we want left to be negative and right to be positive
}

CRadians CTensorSwarmCLAREBot::getActualAgentOrientation(){
    auto positionSensorReading = m_pcPositioningSensor->GetReading();
    CRadians zAngle, yAngle, xAngle;
    const auto orientation = positionSensorReading.Orientation;
    orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    zAngle = Coordinate::ArgosHeadingToOwn(zAngle).SignedNormalize();
    return zAngle;
}

tensorswarm::LaserScan CTensorSwarmCLAREBot::getLaser() const
{
//    const CCI_FootBotLidarSensor::TReadings& laserReadings = m_pcLaser->GetReadings();
    LaserScan laserScan;

    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->agentObject->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        auto distance = std::max(this->agentObject->distance_sensors[sensor_index].getDistance(), (float) this->agentObject->config.DISTANCE_SENSOR_PROXIMITY_RANGE);
        LaserRay laserRay;
        laserRay.value = distance;
        laserRay.angle = sensor_rotation.GetValue();
        laserScan.laser_rays.push_back(laserRay);
        argos::LOG << "Laser ray: " << laserRay.value << " at angle: " << laserRay.angle << std::endl;
    }

//    for (const auto& reads : laserReadings) {
//        LaserRay laserRay;
//        laserRay.value = reads.Value;
//        laserRay.angle = reads.Angle.GetValue();
//        laserScan.laser_rays.push_back(laserRay);
//    }
    return laserScan;
}

geometry_msgs::Twist CTensorSwarmCLAREBot::getVelocities() const
{
    geometry_msgs::Twist twist;

    //Use agentobject wheelspeed and inter wheel distance to calculate linear and angular velocity
    twist.linear.x = this->current_speed;
    twist.angular.z = this->current_angular_speed;
    argos::LOG << "Linear: " << twist.linear.x << " Angular: " << twist.angular.z << std::endl;
    return twist;
}

void CTensorSwarmCLAREBot::setVelocity(const geometry_msgs::Twist& twist) {
    Real v = twist.linear.x;
    Real w = twist.angular.z;

    const auto leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
    const auto rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CTensorSwarmCLAREBot::setNewGoal(const geometry_msgs::Pose2D &new_goal) {
    m_goal = new_goal;
    m_previous_goal_distance = -1.0;
}

geometry_msgs::Pose2D CTensorSwarmCLAREBot::getGoal() const {
    return m_goal;
}
double CTensorSwarmCLAREBot::goalDistance(const argos::CVector3& currentPosition) const{
    return (currentPosition - convertVec(m_goal)).Length();
}

double CTensorSwarmCLAREBot::goalDistance(const geometry_msgs::Pose2D& currentPosition) const {
    return (convertVec(currentPosition) - convertVec(m_goal)).Length();
}

double CTensorSwarmCLAREBot::goalProgress(const argos::CVector3& currentPosition) {
    if(m_goal_progress_called) {
        throw std::logic_error("You are not allowed to call goalProgress() twice in an iteration.");
    }
    m_goal_progress_called = true;

    double result = 0.0;
    if(m_previous_goal_distance >= 0.0) {
        result = m_previous_goal_distance - goalDistance(currentPosition);
    }
    m_previous_goal_distance = goalDistance(currentPosition);
    if(std::fabs(result) > 0.2) {
        throw std::logic_error("Goal progress to large!");
    }
    return result;
}

void CTensorSwarmCLAREBot::Reset() {
    m_arrived = false;
    m_previous_goal_distance = -1.0;
}

REGISTER_CONTROLLER(CTensorSwarmCLAREBot, "tensor_swarm_CLARE_bot_controller")
