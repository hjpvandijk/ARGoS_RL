// Copyright (C) 2018 deeplearningrobotics.ai
#include "ros_service_loop_function_CLARE.h"

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

#include <sys/file.h>
#include <errno.h>

#include <functional>
#include <iostream>

#include "ros_helpers.h"

std::string getNodeName() {
    int i = 0;
    std::string name;
    do {
        name = "ai" + std::to_string(i);
        const std::string filename = "/tmp/argos_" + name + ".pid";
        int pid_file = open(filename.c_str(), O_CREAT | O_RDWR, 0666);
        int rc = flock(pid_file, LOCK_EX | LOCK_NB);
        if(rc) {
            std::cout << "instance running: " << filename << std::endl;
            if(EWOULDBLOCK == errno)
                std::cout << "instance running: " << filename << std::endl;
        }
        else {
            break;
        }
        ++i;
    } while(true);

    return name;
}

ROSServiceLoopFunctionCLARE::ROSServiceLoopFunctionCLARE() : m_service_data_available(false), m_loop_done(false), m_episode_time() {
    int argc = 0;
    char *argv = (char *) "";

    std::string name = getNodeName();

    std::cout << "Node name: " << name << std::endl;
    ros::init(argc, &argv, name);

    m_ros_thread = std::thread([this, name]() {
        ros::NodeHandle n;
        auto service =
                n.advertiseService<tensorswarm::AICLAREServiceRequest, tensorswarm::AICLAREServiceResponse>
                        (name,
                         std::bind(
                                 &ROSServiceLoopFunctionCLARE::ServiceFunction,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2));
        ros::spin();
    });
}

void ROSServiceLoopFunctionCLARE::Init(argos::TConfigurationNode &t_tree) {
    m_episode_time = 0;
}

void ROSServiceLoopFunctionCLARE::Reset() {
    m_episode_time = 0;
}

bool ROSServiceLoopFunctionCLARE::ServiceFunction(const tensorswarm::AICLAREServiceRequest &req,
                                             tensorswarm::AICLAREServiceResponse &resp) {
    std::unique_lock<std::mutex> lk(m_m_main);
    m_req_store = req;
    m_service_data_available = true;
    lk.unlock();
    m_cv_main.notify_one();
    lk.lock();
    while (!m_loop_done) {
        m_cv_service.wait(lk);
    }
    resp = m_resp_store;
    m_loop_done = false;
    m_resp_store = tensorswarm::AICLAREServiceResponse();
    lk.unlock();
    return true;
}

void ROSServiceLoopFunctionCLARE::PreStep() {
    m_lk = std::unique_lock<std::mutex>(m_m_main);

    while (!m_service_data_available) {
        m_cv_main.wait(m_lk);
    }
    m_service_data_available = false;

    auto robots_map = GetSpace().GetEntitiesByType("pipuck");

    if (!m_req_store.reset_poses.empty()) {
        std::size_t i = 5;
        for (const auto &elem : robots_map) {
            auto robot = any_cast<CPiPuckEntity *>(elem.second);

            geometry_msgs::Pose2D pos;
            pos.x = i; pos.y = i;
            auto move_non_colliding = MoveEntity(
                    robot->GetEmbodiedEntity(),  // move the body of the robot
                    convertVec(pos), // to this position
                    convertQuat(pos), // with this orientation
                    false);
            ++i;
        }
    }

    std::size_t i = 0;
    for (const auto &elem : robots_map) {
        auto robot = any_cast<CPiPuckEntity *>(elem.second);
        CTensorSwarmCLAREBot &cController = dynamic_cast<CTensorSwarmCLAREBot &>(robot->GetControllableEntity().GetController());

        if (!m_req_store.reset_poses.empty()) {
            assert(m_req_store.reset_poses.size() == robots_map.size());
            assert(m_req_store.goal_poses.size() == robots_map.size());
            const auto &pos = m_req_store.reset_poses[i];

            robot->Reset();
            auto move_non_colliding = MoveEntity(
                    robot->GetEmbodiedEntity(),  // move the body of the robot
                    convertVec(pos), // to this position
                    convertQuat(pos), // with this orientation
                    false);
            //std::cout << "Reset pose: x " << pos.x << " y " << pos.y << " theta: " << pos.theta << "\n";

            if (!move_non_colliding) {
                std::cerr << "Resetting position caused collision!" << std::endl;
            }

            cController.setNewGoal(m_req_store.goal_poses[i]);
            cController.setVelocity(geometry_msgs::Twist());
            m_episode_time = 0;

        } else {
            assert(robots_map.size() == m_req_store.twists.size());

            cController.setVelocity(m_req_store.twists[i]);
        }
        ++i;
    }
}

void ROSServiceLoopFunctionCLARE::PostStep() {
    argos::LOG << "PostStep" << std::endl;
    using namespace argos;
    auto robots_map = GetSpace().GetEntitiesByType("pipuck");
    auto counter = 0;
    for (const auto &elem : robots_map) {
        //std::cout << "**** ID: " << counter << " ********\n";
        auto robot = any_cast<CPiPuckEntity *>(elem.second);
        const auto &position = robot->GetEmbodiedEntity().GetOriginAnchor().Position;

//        geometry_msgs::Pose2D pose;
//        pose.x = position.GetX();
//        pose.y = position.GetY();

//        CRadians angle;
//        CVector3 angle_axis;
//        robot->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToAngleAxis(angle, angle_axis);
//
//        pose.theta = angle.SignedNormalize().GetValue();
//        pose.theta = std::copysign(pose.theta, angle_axis.GetZ());
//
//        assert(std::abs(angle_axis.GetX()) < 0.01 && std::abs(angle_axis.GetY()) < 0.01);

        CTensorSwarmCLAREBot &cController = dynamic_cast<CTensorSwarmCLAREBot &>(robot->GetControllableEntity().GetController());

        double reward = calculateReward(cController.agentObject.get(), robot);
        constexpr double reward_penalty = -15.0;
        constexpr double arrival_reward = 15.0;
        constexpr double distance_reward = 3.0;
        constexpr double angular_vel_penalty = -0.1;
        constexpr double success_distance = 0.25;

        constexpr uint max_episode_length = 600; // TODO: Get it from service.
        constexpr double speed_reward = 0;

//        bool done = false;
//        //std::cout << "Goal: " << cController.getGoal() << "\n";
//
//        //std::cout << "Distance to goal" << cController.goalDistance(position) << "\n";
//
//        if (/*!cController.arrived() && */cController.goalDistance(position) < success_distance) {
//            reward += arrival_reward;
//            //std::cout << "Arrival reward: " << arrival_reward << "\n";
//
//            reward += speed_reward * (max_episode_length - m_episode_time)/max_episode_length;
//            //std::cout << m_episode_time << "\n";
//            //std::cout << "speed reward: " << speed_reward * (max_episode_length - m_episode_time)/max_episode_length << "\n";
//
//            //done = true;
//
//            cController.setArrived();
//        }
//
//        const auto goal_progress = cController.goalProgress(position);
//        if (goal_progress < 0.0) {
//            reward += distance_reward * goal_progress * 2.0;
//        } else {
//            reward += distance_reward * goal_progress;
//        }
//        //std::cout << "Distance reward: " << distance_reward * goal_progress << std::endl;
//
//
//        if (robot->GetEmbodiedEntity().IsCollidingWithSomething()) {
//            //std::cout << "Collision penalty: " << reward_penalty << std::endl;
//            reward += reward_penalty;
//
//            //done = true;
//
//        }
//        //std::cout << "Final reward " << reward << std::endl;
//
//        // TODO: Check if this improves outcome.
//        /*if (cController.getVelocities().angular.z > 0.7) {
//          //std::cout << "Applying angular velocity large penalty: vel_r: "  << cController
//          //        .getVelocities().angular.z << std::endl;
//
//          reward += angular_vel_penalty*cController.getVelocities().angular.z;
//
//        }*/

        m_resp_store.rewards.push_back(reward);
//        m_resp_store.done.push_back(done);

//        tensorswarm::Observation ob;
        tensorswarm::State state = constructState(&cController);

//        state.pose = pose;
//        state.twist = cController.getVelocities();
//        state.laser_scan = cController.getLaser();
//
////        ob.pose = pose;
////        ob.laser_scan = cController.getLaser();
////        ob.twist = cController.getVelocities();
        m_resp_store.states.push_back(state);

        ++counter;
    }
    ++m_episode_time;

    m_loop_done = true;
    m_lk.unlock();
    m_cv_service.notify_one();

}

tensorswarm::Map ROSServiceLoopFunctionCLARE::getLocalMapForNN(const Agent* agent) {
    tensorswarm::Map map;

    auto cell_size = agent->quadtree->getResolution();
    auto local_size = int(std::min(2 * agent->config.FRONTIER_SEARCH_RADIUS / agent->quadtree->getResolution(), agent->quadtree->getRootBox().size / agent->quadtree->getResolution()));
    map.height = local_size;
    map.width = local_size;
    const auto map_array_size = local_size * local_size;
    float local_map[map_array_size]; // Initialize with default value (unexplored)

    // Define the bounding box for the local map
    float half_size = local_size / 2.0f;
    float x_min = agent->position.x - half_size;
    float y_min = agent->position.y - half_size;
//    float x_max = agent.position.x + half_size;
//    float y_max = agent.position.y + half_size;

    // Query the quadtree for cells within the bounding box
//        auto cells = query_quadtree(agenbt->quadtree, x_min, y_min, x_max, y_max);
    auto boxesAndPheromones = agent->quadtree->queryBoxesAndPheromones(agent->position, local_size, agent->elapsed_ticks / agent->ticks_per_second);

    // Fill the local map with confidence values
    for (const auto& [box, pheromone] : boxesAndPheromones) {
        auto boxCenter = box.getCenter();
        // Calculate the grid indices for the cell
        int x_start = static_cast<int>((boxCenter.x - x_min) / cell_size);
        int y_start = static_cast<int>((boxCenter.y - y_min) / cell_size);
        int x_end = static_cast<int>((boxCenter.x + box.size - x_min) / cell_size);
        int y_end = static_cast<int>((boxCenter.y + box.size - y_min) / cell_size);

        // Fill the local map with the cell's confidence value
        for (int i = x_start; i < x_end; ++i) {
            for (int j = y_start; j < y_end; ++j) {
                if (i >= 0 && i < local_size && j >= 0 && j < local_size) {
                    local_map[i * local_size + j] = pheromone;
                }
            }
        }
    }

    map.data.assign(local_map, local_map + map_array_size);
    return map;
}

tensorswarm::State ROSServiceLoopFunctionCLARE::constructState(CTensorSwarmCLAREBot* controller) {
auto agent = controller->agentObject.get();
tensorswarm::State state;

// 1. Get local map ( grid centered on the robot)
tensorswarm::Map local_map = getLocalMapForNN(agent);
state.map = local_map;

// 2. Get proximity sensor readings
//vec_t proximity_data;
for (int i = 0; i < agent->distance_sensors.size(); i++) {
    auto sensor = agent->distance_sensors[i];
    argos::CRadians sensor_rotation = agent->heading - i * argos::CRadians::PI_OVER_TWO;
    tensorswarm::LaserRay ray;
    ray.value = sensor.getDistance();
    ray.angle = sensor_rotation.GetValue();
    state.laser_scan.laser_rays.push_back(ray);
}

auto position = agent->getPosition();

//3. Get position and orientation
geometry_msgs::Pose2D pose;
pose.x = position.x;
pose.y = position.y;


pose.theta = agent->heading.GetValue(); //In Radians

state.pose = pose;

state.twist = controller->getVelocities();

return state;
}

float ROSServiceLoopFunctionCLARE::calculateReward(Agent* agent, CPiPuckEntity * robot) {
    // Calculate reward based on the map and agent's performance

    //Get map, calculate certainty for each cell. Reward is absolute difference between certainty and 0.5 (ambiguous)
    //Get entire map
    auto boxesAndPheromones = agent->quadtree->getAllBoxes(agent->elapsed_ticks / agent->ticks_per_second);
    //Calculate certainty for each cell: abs(pheromone - 0.5). Combine them using a squared weighted root function.
    //We do this so exploring new cells is highly rewarded, but increasing certainty by a lot is also rewarded.
    float certainty_score = 0.0f;
    for (const auto& [box, pheromone] : boxesAndPheromones) {
        certainty_score += std::sqrt(0.1f * std::abs(pheromone - 0.5)); //TODO: update with new pheromone fitness calculation
    }

    float certainty_score_reward = certainty_score - this->previous_certainty_score;

    float collision_punishment = 0.0f;

    //Punish collisions
    if (robot->GetEmbodiedEntity().IsCollidingWithSomething()) {
        collision_punishment = 10;
    }

    //Get battery level, reward is battery level, so that the agent learns to conserve energy
    float battery_level = agent->batteryManager.battery.charge;
    float battery_score_punishment = this->previous_battery_level - battery_level;

    //Reward is the certainty score minus the battery used
    float reward = certainty_score_reward - battery_score_punishment * 10 - collision_punishment;

    this->previous_certainty_score = certainty_score;
    this->previous_battery_level = battery_level;

    //Get mission duration, reward is duration, so that the agent learns to complete the mission quickly
    //This is implicitly done by the agent, as it will get a higher reward when it explores new areas

    return reward;
}

REGISTER_LOOP_FUNCTIONS(ROSServiceLoopFunctionCLARE, "ROSServiceLoopFunctionCLARE")
