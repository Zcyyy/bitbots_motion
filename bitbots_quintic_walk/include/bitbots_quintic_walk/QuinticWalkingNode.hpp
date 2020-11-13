/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef QUINTICWALKNODE_HPP
#define QUINTICWALKNODE_HPP

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include <stdexcept>
#include <exception>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <bitbots_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
//#include <moveit_msgs/RobotState.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_quintic_walk/WalkingDebug.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_msgs/FootPressure.h>
#include <bitbots_msgs/Fallcheck.h>
#include <bitbots_msgs/SpecialGait.h>

#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/kinematics_base/kinematics_base.h>
//#include <moveit/move_group_interface/move_group_interface.h>

#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
//#include "bitbots_ik/AnalyticIKSolver.hpp"
//#include "bitbots_ik/BioIKSolver.hpp"
#include "bitbots_quintic_walk/WalkEngine.hpp"
#include "DspSDK/DspHandler.h"
#include <std_msgs/Bool.h>
#include <unistd.h>


class QuinticWalkingNode {
public:
    QuinticWalkingNode();

    /**
     * This is the main loop which takes care of stopping and starting of the walking.
     * A small state machine is tracking in which state the walking is and builds the trajectories accordingly.
     */
    void run();

    /**
     * Dynamic reconfigure callback. Takes in new parameters and applies them to the needed software parts
     */
    void reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

    /**
     * Initialize internal WalkEngine to correctly zeroed, usable state
     */
    void initializeEngine();

    bool SetHeadMoveValid(std_srvs::SetBool::Request& req,
                          std_srvs::SetBool::Response& res);

    bool SetSensorEnableValid(std_srvs::SetBool::Request& req,
                              std_srvs::SetBool::Response& res);

    bool ResetOdometry(std_srvs::SetBool::Request& req,
                       std_srvs::SetBool::Response& res);

    bool SetSpecialGaitValid(bitbots_msgs::SpecialGait::Request& req,bitbots_msgs::SpecialGait::Response& res);

/*    bool DoLeftKick(std_srvs::SetBool::Request& req,
                    std_srvs::SetBool::Response& res);

    bool DoRightKick(std_srvs::SetBool::Request& req,
                     std_srvs::SetBool::Response& res);

    bool DoStandFront(std_srvs::SetBool::Request& req,
                      std_srvs::SetBool::Response& res);

    bool DoStandBack(std_srvs::SetBool::Request& req,
                     std_srvs::SetBool::Response& res);

    bool DoLeftSave(std_srvs::SetBool::Request& req,
                        std_srvs::SetBool::Response& res);

    bool DoRightSave(std_srvs::SetBool::Request& req,
                         std_srvs::SetBool::Response& res);

    bool DoWalkKickLeft(std_srvs::Trigger::Request& req,
                         std_srvs::Trigger::Response& res);

    bool DoWalkKickRight(std_srvs::Trigger::Request& req,
                         std_srvs::Trigger::Response& res);
*/
    bool SetTorqueEnable(std_srvs::SetBool::Request& req,
                         std_srvs::SetBool::Response& res);

    bool SetGaitValid(std_srvs::SetBool::Request& req,
                      std_srvs::SetBool::Response& res);

    bool ResetGait(std_srvs::SetBool::Request& req,
                   std_srvs::SetBool::Response& res);

    bool GetSpecialGaitPending(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

    bool GetOdometryResetPending(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);

    bool GaitResetPending(std_srvs::Trigger::Request& req,
                          std_srvs::Trigger::Response& res);

    bool GetWalkKickPending(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res);

   private:
    void publishControllerCommands(std::vector<std::string> joint_names, std::vector<double> positions);

    void publishDebug(tf::Transform &trunk_to_support_foot, tf::Transform &trunk_to_flying_foot);

    void publishMarker(std::string name_space, std::string frame, geometry_msgs::Pose pose, float r, float g, float b,
                       float a);

    void publishMarkers();

    void publishOdometry();
    void publishImuAngle();

    void cmdVelCb(geometry_msgs::Twist msg);
    void Fallen(bitbots_msgs::Fallcheck msg);

    void headPosCb(bitbots_msgs::JointCommand msg);

    void imuCb(bitbots_msgs::Imu msg);

    void pressureCb(bitbots_msgs::FootPressure msg);

    void robStateCb(humanoid_league_msgs::RobotControlState msg);

    //void jointStateCb(sensor_msgs::JointState msg);

    void kickCb(std_msgs::BoolConstPtr msg);

    void cop_l_cb(const geometry_msgs::PointStamped msg);

    void cop_r_cb(const geometry_msgs::PointStamped msg);

    void calculateJointGoals();

    double getTimeDelta();

    void SetDspCommand();
    void ClearDspValidState();
    void GetDataFromDsp();

    bool _fake_mode;
    bool _debugActive;
    bool _simulation_active;

    bool _first_run;

    double _engineFrequency;

    bool _phaseResetActive;
    double _phaseResetPhase;
    double _groundMinPressure;
    bool _copStopActive;
    double _copXThreshold;
    double _copYThreshold;
    bool _pressureStopActive;
    double _ioPressureThreshold;
    double _fbPressureThreshold;

    bool _imuActive;
    double _imu_pitch_threshold;
    double _imu_roll_threshold;
    double _imu_pitch_vel_threshold;
    double _imu_roll_vel_threshold;


    bool _publishOdomTF;
    int _odomPubFactor;
    std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
    double _last_ros_update_time;

    int _robotState;
    int _marker_id;

    bitbots_quintic_walk::WalkingParameter _params;

    Eigen::Vector3d _trunkPos;
    Eigen::Vector3d _trunkAxis;
    Eigen::Vector3d _footPos;
    Eigen::Vector3d _footAxis;
    bool _isLeftSupport;

    bool _special_gait_pending;
    bool _odometry_reset_pending;
    bool _gait_reset_pending;
    bool _walk_kick_pending;

    Eigen::Vector3d _real_velocities;
    Eigen::Vector3d _real_odometry;
    Eigen::Vector3d odom;
    Eigen::Vector3d _imu_angle;
    Eigen::Vector2d _real_head_pos;
    Eigen::Matrix<double, 7, 1> _real_body_pos;

    /**
     * Saves current orders as [x-direction, y-direction, z-rotation]
     */
    Eigen::Vector3d _currentOrders;
    Eigen::Vector2d _headPos;
    Eigen::Vector2i _gaitId;

    /**
     * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
     * Is used to limit _currentOrders to sane values
     */
    Eigen::Vector3d _max_step;

    /**
     * Measures how much distance we can traverse in X and Y direction combined
     */
    double _max_step_xy;
    bitbots_quintic_walk::QuinticWalk _walkEngine;

    bitbots_msgs::JointCommand _command_msg;
    bitbots_msgs::Fallcheck _whitch_stand;
    nav_msgs::Odometry _odom_msg;
    bitbots_msgs::Imu Imu_Angle;
    geometry_msgs::TransformStamped _odom_trans;
    geometry_msgs::TransformStamped _neck_to_head;
    geometry_msgs::TransformStamped _head_to_camera;
    
    int first_time;
    int first_stand;
    int can_head_move;
    double x,y,z;
    ros::NodeHandle _nh;

    ros::Publisher _pubControllerCommand;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubImuAngle;
    ros::Publisher _pubSupport;
    tf::TransformBroadcaster _odom_broadcaster;
    tf2_ros::TransformBroadcaster _head_to_camera_trans;
    tf2_ros::TransformBroadcaster _neck_to_head_trans;
    ros::Publisher _pubDebug;
    ros::Publisher _pubDebugMarker;

    ros::Subscriber _subCmdVel;
    ros::Subscriber _fall;
    ros::Subscriber _subHeadPos;
    ros::Subscriber _subRobState;
    ros::Subscriber _subJointStates;
    ros::Subscriber _subKick;
    ros::Subscriber _subImu;
    ros::Subscriber _subPressure;
    ros::Subscriber _subCopL;
    ros::Subscriber _subCopR;

    // dsp command service
    ros::ServiceServer _set_head_move_valid_service;
    ros::ServiceServer _set_sensor_enable_valid_service;
    ros::ServiceServer _set_special_gait_valid_service;
    ros::ServiceServer _do_left_kick_service;
    ros::ServiceServer _do_right_kick_service;
    ros::ServiceServer _do_stand_front_service;
    ros::ServiceServer _do_stand_back_service;
    ros::ServiceServer _do_left_save_service;
    ros::ServiceServer _do_right_save_service;
    ros::ServiceServer _do_walk_kick_left_service;
    ros::ServiceServer _do_walk_kick_right_service;
    ros::ServiceServer _set_torque_enable_service;
    ros::ServiceServer _set_gait_valid_service;
    ros::ServiceServer _reset_odometry_service;
    ros::ServiceServer _reset_gait_service;

    ros::ServiceServer _get_special_gait_pending_service;
    ros::ServiceServer _get_odometry_reset_pending_service;
    ros::ServiceServer _gait_reset_pending_service;
    ros::ServiceServer _get_walk_kick_pending_service;

    geometry_msgs::PointStamped _cop_l;
    geometry_msgs::PointStamped _cop_r;

    // MoveIt!
    /*robot_model_loader::RobotModelLoader _robot_model_loader;
    robot_model::RobotModelPtr _kinematic_model;
    robot_state::RobotStatePtr _goal_state;
    robot_state::RobotStatePtr _current_state;
    const robot_state::JointModelGroup *_all_joints_group;
    const robot_state::JointModelGroup *_legs_joints_group;
    const robot_state::JointModelGroup *_lleg_joints_group;
    const robot_state::JointModelGroup *_rleg_joints_group;*/

    // IK solver
    //bitbots_ik::BioIKSolver _bioIK_solver;
    std::shared_ptr<DspSDK::DspHandler> _dsp_handler;
};

#endif
