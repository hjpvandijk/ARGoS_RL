// Copyright (C) 2018 deeplearningrobotics.ai
#include "ros_service_loop_function_CLARE.h"

#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

#include <sys/file.h>
#include <errno.h>

#include <functional>
#include <iostream>

#include "tensor_swarm_CLARE_bot.h"
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

    using namespace argos;
    auto robots_map = GetSpace().GetEntitiesByType("pipuck");
    auto counter = 0;
    for (const auto &elem : robots_map) {
        //std::cout << "**** ID: " << counter << " ********\n";
        auto robot = any_cast<CPiPuckEntity *>(elem.second);
        const auto &position = robot->GetEmbodiedEntity().GetOriginAnchor().Position;

        geometry_msgs::Pose2D pose;
        pose.x = position.GetX();
        pose.y = position.GetY();

        CRadians angle;
        CVector3 angle_axis;
        robot->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToAngleAxis(angle, angle_axis);

        pose.theta = angle.SignedNormalize().GetValue();
        pose.theta = std::copysign(pose.theta, angle_axis.GetZ());

        assert(std::abs(angle_axis.GetX()) < 0.01 && std::abs(angle_axis.GetY()) < 0.01);

        CTensorSwarmCLAREBot &cController = dynamic_cast<CTensorSwarmCLAREBot &>(robot->GetControllableEntity().GetController());

        double reward = 0.0;
        constexpr double reward_penalty = -15.0;
        constexpr double arrival_reward = 15.0;
        constexpr double distance_reward = 3.0;
        constexpr double angular_vel_penalty = -0.1;
        constexpr double success_distance = 0.25;

        constexpr uint max_episode_length = 600; // TODO: Get it from service.
        constexpr double speed_reward = 0;

        bool done = false;
        //std::cout << "Goal: " << cController.getGoal() << "\n";

        //std::cout << "Distance to goal" << cController.goalDistance(position) << "\n";

        if (/*!cController.arrived() && */cController.goalDistance(position) < success_distance) {
            reward += arrival_reward;
            //std::cout << "Arrival reward: " << arrival_reward << "\n";

            reward += speed_reward * (max_episode_length - m_episode_time)/max_episode_length;
            //std::cout << m_episode_time << "\n";
            //std::cout << "speed reward: " << speed_reward * (max_episode_length - m_episode_time)/max_episode_length << "\n";

            //done = true;

            cController.setArrived();
        }

        const auto goal_progress = cController.goalProgress(position);
        if (goal_progress < 0.0) {
            reward += distance_reward * goal_progress * 2.0;
        } else {
            reward += distance_reward * goal_progress;
        }
        //std::cout << "Distance reward: " << distance_reward * goal_progress << std::endl;


        if (robot->GetEmbodiedEntity().IsCollidingWithSomething()) {
            //std::cout << "Collision penalty: " << reward_penalty << std::endl;
            reward += reward_penalty;

            //done = true;

        }
        //std::cout << "Final reward " << reward << std::endl;

        // TODO: Check if this improves outcome.
        /*if (cController.getVelocities().angular.z > 0.7) {
          //std::cout << "Applying angular velocity large penalty: vel_r: "  << cController
          //        .getVelocities().angular.z << std::endl;

          reward += angular_vel_penalty*cController.getVelocities().angular.z;

        }*/

        m_resp_store.rewards.push_back(reward);
        m_resp_store.done.push_back(done);

//        tensorswarm::Observation ob;
        tensorswarm::State state;

        state.pose = pose;
        state.twist = cController.getVelocities();
        state.laser_scan = cController.getLaser();

//        ob.pose = pose;
//        ob.laser_scan = cController.getLaser();
//        ob.twist = cController.getVelocities();
        m_resp_store.states.push_back(state);

        ++counter;
    }
    ++m_episode_time;

    m_loop_done = true;
    m_lk.unlock();
    m_cv_service.notify_one();

}

REGISTER_LOOP_FUNCTIONS(ROSServiceLoopFunctionCLARE, "ROSServiceLoopFunctionCLARE")
