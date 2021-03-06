#include "bitbots_quintic_walk/QuinticWalkingNode.hpp"

constexpr int StandUp = 0;
constexpr int kDoLeftKick = 1;
constexpr int kDoRightKick = 2;
constexpr int kDoStandFront = 3;
constexpr int kDoStandBack = 4;
constexpr int kDoLeftSave = 5;
constexpr int kDoRightSave = 6;
constexpr int init_stand = 10;
//_robot_model_loader("/robot_description", false),

QuinticWalkingNode::QuinticWalkingNode()
    : _special_gait_pending(false),
      _odometry_reset_pending(false),
      _gait_reset_pending(false),
      _walk_kick_pending(false) {
    // init variables
    _robotState = humanoid_league_msgs::RobotControlState::CONTROLABLE;
    //_walkEngine = bitbots_quintic_walk::QuinticWalk();
    _isLeftSupport = true;
    _currentOrders[0] = 0.0;
    _currentOrders[1] = 0.0;
    _currentOrders[2] = 0.0;

    _marker_id = 1;
    first_time = 1;
    first_stand = 1;
    x = 0.0;
    y = 0.0;
    z = 0.043;
    can_head_move = 0;
    _odom_broadcaster = tf::TransformBroadcaster();
    static tf2_ros::TransformBroadcaster _head_to_base_footprint;

    // read config
    _nh.param<double>("engineFrequency", _engineFrequency, 100.0);
    _nh.param<bool>("/simulation_active", _simulation_active, false);
    _nh.param<bool>("/walking/publishOdomTF", _publishOdomTF, true);
    _nh.param<bool>("/walking/fake_mode", _fake_mode, false);

    /* init publisher and subscriber */
    _command_msg = bitbots_msgs::JointCommand();
    _pubControllerCommand =
        _nh.advertise<bitbots_msgs::JointCommand>("walking_motor_goals", 1);
    _odom_msg = nav_msgs::Odometry();
    _pubOdometry = _nh.advertise<nav_msgs::Odometry>("walk_odometry", 1);
    _pubImuAngle = _nh.advertise<bitbots_msgs::Imu>("imu/data",1);
    _pubSupport = _nh.advertise<std_msgs::Char>("walk_support_state", 1);
    _subCmdVel = _nh.subscribe("cmd_vel", 1, &QuinticWalkingNode::cmdVelCb, this,
                               ros::TransportHints().tcpNoDelay());
    _subHeadPos =
        _nh.subscribe("head_motor_goals", 1, &QuinticWalkingNode::headPosCb, this,
                      ros::TransportHints().tcpNoDelay());
    _subRobState =
        _nh.subscribe("robot_state", 1, &QuinticWalkingNode::robStateCb, this,
                      ros::TransportHints().tcpNoDelay());
    // todo not really needed
    //_subJointStates = _nh.subscribe("joint_states", 1,
    //&QuinticWalkingNode::jointStateCb, this,
    //ros::TransportHints().tcpNoDelay());
    _subKick = _nh.subscribe("kick", 1, &QuinticWalkingNode::kickCb, this,
                             ros::TransportHints().tcpNoDelay());
    //_subImu = _nh.subscribe("imu/data", 1, &QuinticWalkingNode::imuCb, this,
    //                        ros::TransportHints().tcpNoDelay());
    //_subPressure = _nh.subscribe("foot_pressure_filtered", 1,
    //                             &QuinticWalkingNode::pressureCb, this,
    //                             ros::TransportHints().tcpNoDelay());
    _fall = _nh.subscribe("fallen",1,&QuinticWalkingNode::Fallen, this, ros::TransportHints().tcpNoDelay());
    _subCopL = _nh.subscribe("cop_l", 1, &QuinticWalkingNode::cop_l_cb, this,
                             ros::TransportHints().tcpNoDelay());
    _subCopR = _nh.subscribe("cop_r", 1, &QuinticWalkingNode::cop_r_cb, this,
                             ros::TransportHints().tcpNoDelay());

    /* debug publisher */
    _pubDebug =
        _nh.advertise<bitbots_quintic_walk::WalkingDebug>("walk_debug", 1);
    _pubDebugMarker =
        _nh.advertise<visualization_msgs::Marker>("walk_debug_marker", 1);

    // load MoveIt! model
    //_robot_model_loader.loadKinematicsSolvers(
    //    kinematics_plugin_loader::KinematicsPluginLoaderPtr(
    //        new kinematics_plugin_loader::KinematicsPluginLoader()));
    //_kinematic_model = _robot_model_loader.getModel();
    //_all_joints_group = _kinematic_model->getJointModelGroup("All");
    //_legs_joints_group = _kinematic_model->getJointModelGroup("Legs");
    //_lleg_joints_group = _kinematic_model->getJointModelGroup("LeftLeg");
    //_rleg_joints_group = _kinematic_model->getJointModelGroup("RightLeg");
    //_goal_state.reset(new robot_state::RobotState(_kinematic_model));
    //_goal_state->setToDefaultValues();
    // we have to set some good initial position in the goal state, since we are
    // using a gradient based method. Otherwise, the first step will be not
    // correct
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch",
                                          "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    //for (int i = 0; i < names_vec.size(); i++) {
      // besides its name, this method only changes a single joint position...
     //_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    //}

    //_current_state.reset(new robot_state::RobotState(_kinematic_model));
    //_current_state->setToDefaultValues();

    // initilize IK solver
    /*_bioIK_solver = bitbots_ik::BioIKSolver(
        *_all_joints_group, *_lleg_joints_group, *_rleg_joints_group);
    _bioIK_solver.set_use_approximate(true);*/

    _first_run = true;

    _set_head_move_valid_service = _nh.advertiseService(
        "set_head_move_valid", &QuinticWalkingNode::SetHeadMoveValid, this);
    _set_sensor_enable_valid_service = _nh.advertiseService(
        "set_sensor_enable_valid", &QuinticWalkingNode::SetSensorEnableValid, this);
    _set_special_gait_valid_service = _nh.advertiseService(
        "set_special_gait_valid", &QuinticWalkingNode::SetSpecialGaitValid, this);
/*    _do_left_kick_service = _nh.advertiseService(
        "do_left_kick", &QuinticWalkingNode::DoLeftKick, this);
    _do_right_kick_service = _nh.advertiseService(
        "do_right_kick", &QuinticWalkingNode::DoRightKick, this);
    _do_stand_front_service = _nh.advertiseService(
        "do_stand_front", &QuinticWalkingNode::DoStandFront, this);
    _do_stand_back_service = _nh.advertiseService(
        "do_stand_back", &QuinticWalkingNode::DoStandBack, this);
    _do_left_save_service = _nh.advertiseService(
        "do_left_save", &QuinticWalkingNode::DoLeftSave, this);
    _do_right_save_service = _nh.advertiseService(
        "do_right_save", &QuinticWalkingNode::DoRightSave, this);
    _do_walk_kick_left_service = _nh.advertiseService(
        "do_walk_kick_left", &QuinticWalkingNode::DoWalkKickLeft, this);
    _do_walk_kick_right_service = _nh.advertiseService(
        "do_walk_kick_right", &QuinticWalkingNode::DoWalkKickRight, this);*/
    _set_torque_enable_service = _nh.advertiseService(
        "set_torque_enable", &QuinticWalkingNode::SetTorqueEnable, this);
    _set_gait_valid_service = _nh.advertiseService(
        "set_gait_valid", &QuinticWalkingNode::SetGaitValid, this);
    _reset_odometry_service = _nh.advertiseService(
        "reset_odometry", &QuinticWalkingNode::ResetOdometry, this);
    _reset_gait_service = _nh.advertiseService(
        "reset_gait_service", &QuinticWalkingNode::ResetGait, this);

    _get_special_gait_pending_service = _nh.advertiseService(
        "get_special_gait_pending", &QuinticWalkingNode::GetSpecialGaitPending, this);
    _get_odometry_reset_pending_service = _nh.advertiseService(
        "get_odometry_reset_pending", &QuinticWalkingNode::GetOdometryResetPending, this);
    _gait_reset_pending_service = _nh.advertiseService(
        "gait_reset_pending", &QuinticWalkingNode::GaitResetPending, this);
    _get_walk_kick_pending_service = _nh.advertiseService(
        "get_walk_kick_pending", &QuinticWalkingNode::GetWalkKickPending, this);

    // initilize DSP handler
    if (_fake_mode)
      _dsp_handler = std::make_shared<DspSDK::DspHandler>("/dev/ttyTHS2", true);
    else
      _dsp_handler = std::make_shared<DspSDK::DspHandler>("/dev/ttyTHS2");
    _dsp_handler->Init();
    _dsp_handler->SetVelocity(0., 0., 0.);
    _dsp_handler->SetHeadPos(0., 0.);
    _dsp_handler->SetGaitValid(true);
    _dsp_handler->SetTorqueEnable(true);
    _dsp_handler->SetSensorEnableValid(true);
}


void QuinticWalkingNode::run() {
    int odom_counter = 0;

    while (ros::ok()) {
        ros::Rate loopRate(_engineFrequency);
        double dt = getTimeDelta();

        if (_robotState == humanoid_league_msgs::RobotControlState::FALLING) {
            // the robot fell, we have to reset everything and do nothing else
            _walkEngine.reset();
	    //send standup
        } else {
            // we don't want to walk, even if we have orders, if we are not in the right state
            /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
             * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
            // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
            bool walkableState = _robotState == humanoid_league_msgs::RobotControlState::CONTROLABLE ||
                                 _robotState == humanoid_league_msgs::RobotControlState::WALKING
                                 || _robotState == humanoid_league_msgs::RobotControlState::MOTOR_OFF;
            // see if the walk engine has new goals for us
            bool newGoals = _walkEngine.updateState(dt, _currentOrders, walkableState);
            //if (_walkEngine.getState() != "idle") { //todo
                //calculateJointGoals();
		//do something
            //}
        }
        SetDspCommand();
        _dsp_handler->DspThread();
        ClearDspValidState();
	if(first_stand)
	{
	   _real_odometry = _dsp_handler->GetOdometry();
   	   //std::cout << _real_odometry[0] << " " << _real_odometry[1] << " " << _real_odometry[2] << std::endl;
	   odom[0] = _real_odometry[0];
	   odom[1] = _real_odometry[1];
	   odom[2] = _real_odometry[2];
	   _dsp_handler->SetSpecialGaitValid(false);
	}
        GetDataFromDsp();
	
	//tf2::Quaternion pan = _neck_to_head.transform.getRotation();
	//tf2::Quaternion tilt = _head_to_camera.transform.getRotation();
        // publish odometry
        odom_counter++;
        if (odom_counter > _odomPubFactor) {
            publishOdometry();
            odom_counter = 0;
        }
	publishImuAngle();
	if(first_stand)
	{
		_dsp_handler->SetHeadMoveValid(false);
		//_dsp_handler->SetSpecialGaitValid(false);
		first_stand = 0;
	}
	/*if(first_time)
	{
		_head_to_camera.header.stamp = ros::Time::now();
		_head_to_camera.header.frame_id = "neck";
		_head_to_camera.child_frame_id = "head";
		_head_to_camera.transform.translation.x = 0;
		_head_to_camera.transform.translation.y = 0;
		_head_to_camera.transform.translation.z = 0.035;
		tf2::Quaternion q;
		q.setRPY(0,0,0);
		_head_to_camera.transform.rotation.x = q.x();
		_head_to_camera.transform.rotation.y = q.y();
		_head_to_camera.transform.rotation.z = q.z();
		_head_to_camera.transform.rotation.w = 1.0;
		_head_to_camera_trans.sendTransform(_head_to_camera);
	    
		_neck_to_head.header.stamp = ros::Time::now();
		_neck_to_head.header.frame_id = "base_link";
		_neck_to_head.child_frame_id = "neck";
		_neck_to_head.transform.translation.x = 0.0368;//0
		_neck_to_head.transform.translation.y = 0;
		_neck_to_head.transform.translation.z = 0.128;//.167
		tf2::Quaternion q1;
		q1.setRPY(0,0,0);
		_neck_to_head.transform.rotation.x = q1.x();
		_neck_to_head.transform.rotation.y = q1.y();
		_neck_to_head.transform.rotation.z = q1.z();
		_neck_to_head.transform.rotation.w = 1.0;
    		_neck_to_head_trans.sendTransform(_neck_to_head);
	}
	else
	{*/
	if(!first_time)
	{
		_head_to_camera.header.stamp = ros::Time::now();
		_head_to_camera.header.frame_id = "neck";
		_head_to_camera.child_frame_id = "head";
		_head_to_camera.transform.translation.x = 0;
		_head_to_camera.transform.translation.y = 0;
		_head_to_camera.transform.translation.z = 0.035;
		tf2::Quaternion q;
		q.setRPY(0,_headPos[0]*0.1125*3.1415926/180,0);
		_head_to_camera.transform.rotation.x = q.x();
		_head_to_camera.transform.rotation.y = q.y();
		_head_to_camera.transform.rotation.z = q.z();
		_head_to_camera.transform.rotation.w = q.w();
		_head_to_camera_trans.sendTransform(_head_to_camera);
	    	//std::cout << "_headPos[0] is: " << _headPos[0]*0.1125 << std::endl; 
		//std::cout << "x_bias is: " << x << " y_bias is:" << y <<" z_bias is: "<< z << std::endl;

		_neck_to_head.header.stamp = ros::Time::now();
		_neck_to_head.header.frame_id = "base_link";
		_neck_to_head.child_frame_id = "neck";
		_neck_to_head.transform.translation.x = 0.0368;//0
		_neck_to_head.transform.translation.y = 0;
		_neck_to_head.transform.translation.z = 0.128;//.167
		tf2::Quaternion q1;
		q1.setRPY(0,0,_headPos[1]*0.1125*3.1415926/180);
		_neck_to_head.transform.rotation.x = q1.x();
		_neck_to_head.transform.rotation.y = q1.y();
		_neck_to_head.transform.rotation.z = q1.z();
		_neck_to_head.transform.rotation.w = q1.w();
		_neck_to_head_trans.sendTransform(_neck_to_head);
		//std::cout << "_headPos[1] is: " << (_headPos[1]*0.1125)<< std::endl;
	}

        ros::spinOnce();
        loopRate.sleep();
    }
}

/*void QuinticWalkingNode::calculateJointGoals() {
    
    This method computes the next motor goals and publishes them.
    

    // read the cartesian positions and orientations for trunk and fly foot
    //_walkEngine.computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf::Vector3 tf_vec;
    tf::vectorEigenToTF(_trunkPos, tf_vec);
    tf::Quaternion tf_quat = tf::Quaternion();
    tf_quat.setRPY(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
    tf_quat.normalize();
    tf::Transform support_foot_to_trunk(tf_quat, tf_vec);
    tf::Transform trunk_to_support_foot_goal = support_foot_to_trunk.inverse();

    tf::vectorEigenToTF(_footPos, tf_vec);
    tf_quat.setRPY(_footAxis[0], _footAxis[1], _footAxis[2]);
    tf_quat.normalize();
    tf::Transform support_to_flying_foot(tf_quat, tf_vec);
    tf::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * support_to_flying_foot;

    // call ik solver
    bool success = _bioIK_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal,
                                       _walkEngine.getFootstep().isLeftSupport(), _goal_state);

    // publish goals if sucessfull
    if (success) {
        std::vector<std::string> joint_names = _legs_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        _goal_state->copyJointGroupPositions(_legs_joints_group, joint_goals);
        publishControllerCommands(joint_names, joint_goals);
    }

    // publish current support state
    std_msgs::Char support_state;
    if (_walkEngine.isDoubleSupport()) {
        support_state.data = 'd';
    } else if (_walkEngine.isLeftSupport()) {
        support_state.data = 'l';
    } else {
        support_state.data = 'r';
    }
    _pubSupport.publish(support_state);

    // publish debug information
    if (_debugActive) {
        publishDebug(trunk_to_support_foot_goal, trunk_to_flying_foot_goal);
        publishMarkers();
    }

}*/

double QuinticWalkingNode::getTimeDelta() {
    // compute time delta depended if we are currently in simulation or reality
    double dt;
    if (!_simulation_active) {
        std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
        // only take real time difference if walking was not stopped before
        // using c++ time since it is more performant than ros time. We only need a local difference, so it doesnt matter as long as we are not simulating
        // TODO Is this really necessary?
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - _last_update_time);
        dt = time_diff_ms.count() / 1000.0;
        if (dt == 0) {
            ROS_WARN("dt was 0");
            dt = 0.001;
        }
        _last_update_time = current_time;
    } else {
        ROS_WARN_ONCE("Simulation active, using ROS time");
        // use ros time for simulation
        double current_ros_time = ros::Time::now().toSec();
        dt = current_ros_time - _last_ros_update_time;
        _last_ros_update_time = current_ros_time;
    }
    // time is wrong when we run it for the first time
    if (_first_run) {
        _first_run = false;
        dt = 0.0001;
    }
    return dt;
}

void QuinticWalkingNode::cmdVelCb(const geometry_msgs::Twist msg) {
    // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
    // other axis. 

    // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a double step
    // factor 2 since the order distance is only for a single step, not double step
    /*
    double factor = (1.0 / (_params.freq)) / 2.0;
    _currentOrders = {msg.linear.x * factor, msg.linear.y * factor, msg.angular.z * factor};

    // the orders should not extend beyond a maximal step size
    for (int i = 0; i < 3; i++) {
        _currentOrders[i] = std::max(std::min(_currentOrders[i], _max_step[i]), _max_step[i] * -1);
    }
    // translational orders (x+y) should not exceed combined limit. scale if necessary
    if (_max_step_xy != 0) {
        double scaling_factor = (_currentOrders[0] + _currentOrders[1]) / _max_step_xy;
        for (int i = 0; i < 2; i++) {
            _currentOrders[i] = _currentOrders[i] / std::max(scaling_factor, 1.0);
        }
    }

    // warn user that speed was limited
    if (msg.linear.x * factor != _currentOrders[0] ||
        msg.linear.y * factor != _currentOrders[1] ||
        msg.angular.z * factor != _currentOrders[2]) {
        ROS_WARN("Speed command was x: %.2f y: %.2f z: %.2f xy: %.2f but maximum is x: %.2f y: %.2f z: %.2f xy: %.2f",
                 msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.x + msg.linear.y, _max_step[0] / factor,
                 _max_step[1] / factor, _max_step[2] / factor, _max_step_xy / factor);
    }*/

    _currentOrders[0] = msg.linear.x;
    _currentOrders[1] = msg.linear.y;
    _currentOrders[2] = msg.angular.z;
    std::cout<< "x speed is: " << _currentOrders[0] << " y speed is: " << _currentOrders[1] << " z speed is: " << _currentOrders[2] << std::endl;
}

void QuinticWalkingNode::headPosCb(const bitbots_msgs::JointCommand msg) {
    // we use only 2 values from the JointCommand position messages, positions[0, 1] as yaw, pitch in rad
    if(can_head_move)
    {
       _headPos[0] = msg.positions[1]/0.1125;
       _headPos[1] = msg.positions[0]/0.1125;
       first_time = 0;
    }
       //std::cout << "_headPos[0] is: " << _headPos[0] << " _headPos[1] is: " << _headPos[1] << std::endl;
}
/*
void QuinticWalkingNode::imuCb(const bitbots_msgs::Imu msg) {
    if (_imuActive) {
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // compute the pitch offset to the currently wanted pitch of the engine
        double wanted_pitch =
                _params.trunkPitch + _params.trunkPitchPCoefForward * _walkEngine.getFootstep().getNext().x()
                + _params.trunkPitchPCoefTurn * fabs(_walkEngine.getFootstep().getNext().z());
        pitch = pitch + wanted_pitch;

        // get angular velocities
        double roll_vel = msg.angular_velocity.x;
        double pitch_vel = msg.angular_velocity.y;
        if (abs(roll) > _imu_roll_threshold || abs(pitch) > _imu_pitch_threshold ||
            abs(pitch_vel) > _imu_pitch_vel_threshold || abs(roll_vel) > _imu_roll_vel_threshold) {
            _walkEngine.requestPause();
            if (abs(roll) > _imu_roll_threshold) {
                ROS_WARN("imu roll angle stop");
            } else if (abs(pitch) > _imu_pitch_threshold) {
                ROS_WARN("imu pitch angle stop");
            } else if (abs(pitch_vel) > _imu_pitch_vel_threshold) {
                ROS_WARN("imu roll vel stop");
            } else {
                ROS_WARN("imu pitch vel stop");
            }
        }
    }
}*/

/*void QuinticWalkingNode::pressureCb(const bitbots_msgs::FootPressure msg) { // TODO Remove this method since cop_cb is now used
    // we just want to look at the support foot. choose the 4 values from the message accordingly
    // s = support, n = not support, i = inside, o = outside, f = front, b = back
    double sob;
    double sof;
    double sif;
    double sib;

    double nob;
    double nof;
    double nif;
    double nib;

    if (_walkEngine.isLeftSupport()) {
        sob = msg.l_l_b;
        sof = msg.l_l_f;
        sif = msg.l_r_f;
        sib = msg.l_r_b;

        nib = msg.r_l_b;
        nif = msg.r_l_f;
        nof = msg.r_r_f;
        nob = msg.r_r_b;
    } else {
        sib = msg.r_l_b;
        sif = msg.r_l_f;
        sof = msg.r_r_f;
        sob = msg.r_r_b;

        nob = msg.l_l_b;
        nof = msg.l_l_f;
        nif = msg.l_r_f;
        nib = msg.l_r_b;
    }

    // sum to get overall pressure on foot
    double s_sum = sob + sof + sif + sib;
    double n_sum = nob + nof + nif + nib;

    // ratios between pressures to get relative position of CoP
    double s_io_ratio = 100;
    if (sof + sob != 0) {
        s_io_ratio = (sif + sib) / (sof + sob);
        if (s_io_ratio == 0) {
            s_io_ratio = 100;
        }
    }
    double s_fb_ratio = 100;
    if (sib + sob != 0) {
        s_fb_ratio = (sif + sof) / (sib + sob);
        if (s_fb_ratio == 0) {
            s_fb_ratio = 100;
        }
    }

    // check for early step end
    // phase has to be far enough (almost at end of step) to have right foot lifted
    // foot has to have ground contact
    double phase = _walkEngine.getPhase();
    if (_phaseResetActive && ((phase > 0.5 - _phaseResetPhase && phase < 0.5) || (phase > 1 - _phaseResetPhase)) &&
        n_sum > _groundMinPressure) {
        ROS_WARN("Phase resetted!");
        _walkEngine.endStep();
    }

    // check if robot is unstable and should pause
    // this is true if the robot is falling to the outside or to front or back
    if (_pressureStopActive && (s_io_ratio > _ioPressureThreshold || 1 / s_io_ratio > _ioPressureThreshold ||
                                1 / s_fb_ratio > _fbPressureThreshold || s_fb_ratio > _fbPressureThreshold)) {
        _walkEngine.requestPause();

        //TODO this is debug
        if (s_io_ratio > _ioPressureThreshold || 1 / s_io_ratio > _ioPressureThreshold) {
            ROS_WARN("CoP io stop!");
        } else {
            ROS_WARN("CoP fb stop!");
        }
    }

    // decide which CoP
    geometry_msgs::PointStamped cop;
    if (_walkEngine.isLeftSupport()) {
        cop = _cop_l;
    } else {
        cop = _cop_r;
    }

    if (_copStopActive && (abs(cop.point.x) > _copXThreshold || abs(cop.point.y) > _copYThreshold)) {
        _walkEngine.requestPause();
        if (abs(cop.point.x) > _copXThreshold) {
            ROS_WARN("cop x stop");
        } else {
            ROS_WARN("cop y stop");
        }
    }


}*/
//not use
void QuinticWalkingNode::Fallen(const bitbots_msgs::Fallcheck msg){
	std::cout << "fallen type: " << (int)msg.FallenType << std::endl;
	if(msg.FallenType == 0 || msg.FallenType == 2)
	{
		_dsp_handler->SetSpecialGaitValid(false);
		ros::Duration(1).sleep();
		_dsp_handler->SetSpecialGaitValid(true);
		_dsp_handler->SetSpecialGaitId(3, 1);
		ros::Duration(10).sleep();
		_dsp_handler->SetSpecialGaitValid(false);
	}
	else if(msg.FallenType == 1 || msg.FallenType == 3)
	{
		_dsp_handler->SetSpecialGaitValid(false);
		ros::Duration(1).sleep();
		_dsp_handler->SetSpecialGaitValid(true);
		_dsp_handler->SetSpecialGaitId(4, 1);
		ros::Duration(10).sleep();
		_dsp_handler->SetSpecialGaitValid(false);
	}
}

void QuinticWalkingNode::robStateCb(const humanoid_league_msgs::RobotControlState msg) {
    _robotState = msg.state;
}

/*void QuinticWalkingNode::jointStateCb(const sensor_msgs::JointState msg) {
    std::vector<std::string> names_vec = msg.name;
    std::string *names = names_vec.data();

    _current_state->setJointPositions(*names, msg.position.data());
}*/

void QuinticWalkingNode::kickCb(const std_msgs::BoolConstPtr msg) {
    _walkEngine.requestKick(msg->data);
}

void QuinticWalkingNode::cop_l_cb(const geometry_msgs::PointStamped msg) {
    _cop_l = msg;
}

void QuinticWalkingNode::cop_r_cb(const geometry_msgs::PointStamped msg) {
    _cop_r = msg;
}

void
QuinticWalkingNode::reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level) {
    _params.freq = config.freq;
    _params.doubleSupportRatio = config.doubleSupportRatio;
    _params.footDistance = config.footDistance;
    _params.footRise = config.footRise;
    _params.footZPause = config.footZPause;
    _params.footPutDownZOffset = config.footPutDownZOffset;
    _params.footPutDownPhase = config.footPutDownPhase;
    _params.footApexPhase = config.footApexPhase;
    _params.footOvershootRatio = config.footOvershootRatio;
    _params.footOvershootPhase = config.footOvershootPhase;
    _params.trunkHeight = config.trunkHeight;
    _params.trunkPitch = config.trunkPitch;
    _params.trunkPhase = config.trunkPhase;
    _params.trunkXOffset = config.trunkXOffset;
    _params.trunkYOffset = config.trunkYOffset;
    _params.trunkSwing = config.trunkSwing;
    _params.trunkPause = config.trunkPause;
    _params.trunkXOffsetPCoefForward = config.trunkXOffsetPCoefForward;
    _params.trunkXOffsetPCoefTurn = config.trunkXOffsetPCoefTurn;
    _params.trunkPitchPCoefForward = config.trunkPitchPCoefForward;
    _params.trunkPitchPCoefTurn = config.trunkPitchPCoefTurn;

    _params.firstStepSwingFactor = config.firstStepSwingFactor;

    _params.kickLength = config.kickLength;
    _params.kickPhase = config.kickPhase;
    _params.footPutDownRollOffset = config.footPutDownRollOffset;
    _params.kickVel = config.kickVel;

    _walkEngine.setParameters(_params);
    //_bioIK_solver.set_bioIK_timeout(config.bioIKTime);

    _debugActive = config.debugActive;
    _engineFrequency = config.engineFreq;
    _odomPubFactor = config.odomPubFactor;

    _max_step[0] = config.maxStepX;
    _max_step[1] = config.maxStepY;
    _max_step[2] = config.maxStepZ;
    _max_step_xy = config.maxStepXY;

    _imuActive = config.imuActive;
    _imu_pitch_threshold = config.imuPitchThreshold;
    _imu_roll_threshold = config.imuRollThreshold;
    _imu_pitch_vel_threshold = config.imuPitchVelThreshold;
    _imu_roll_vel_threshold = config.imuRollVelThreshold;

    _phaseResetActive = config.phaseResetActive;
    _phaseResetPhase = config.phaseResetPhase;
    _groundMinPressure = config.groundMinPressure;
    _copStopActive = config.copStopActive;
    _copXThreshold = config.copXThreshold;
    _copYThreshold = config.copYThreshold;
    _pressureStopActive = config.pressureStopActive;
    _ioPressureThreshold = config.ioPressureThreshold;
    _fbPressureThreshold = config.fbPressureThreshold;
    _params.pauseDuration = config.pauseDuration;
}


void
QuinticWalkingNode::publishControllerCommands(std::vector<std::string> joint_names, std::vector<double> positions) {
    // publishes the commands to the GroupedPositionController
    _command_msg.header.stamp = ros::Time::now();
    _command_msg.joint_names = joint_names;
    _command_msg.positions = positions;
    std::vector<double> ones(joint_names.size(), -1.0);
    std::vector<double> vels(joint_names.size(), -1.0);
    std::vector<double> accs(joint_names.size(), -1.0);
    std::vector<double> pwms(joint_names.size(), -1.0);
    _command_msg.velocities = vels;
    _command_msg.accelerations = accs;
    _command_msg.max_currents = pwms;

    _pubControllerCommand.publish(_command_msg);
}

void QuinticWalkingNode::publishImuAngle() {
    Imu_Angle.header.stamp = ros::Time::now();
    Imu_Angle.header.frame_id = "IMU";
    Imu_Angle.x = _imu_angle[0];
    Imu_Angle.y = _imu_angle[1];
    Imu_Angle.z = _imu_angle[2];
    _pubImuAngle.publish(Imu_Angle);
}

void QuinticWalkingNode::publishOdometry() {

    // odometry to trunk is transform to support foot * transform from support to trunk    
    tf::Vector3 pos;

    pos[0] = _real_odometry[0] - odom[0];
    pos[1] = _real_odometry[1] - odom[1];
    pos[2] = _real_odometry[2] - odom[2];
    

    // send the odometry also as message
    ros::Time current_time = ros::Time::now();
    _odom_msg.header.stamp = current_time;
    _odom_msg.header.frame_id = "base_link";
    _odom_msg.pose.pose.position.x = pos[0];
    _odom_msg.pose.pose.position.y = pos[1];
    _odom_msg.pose.pose.position.z = 0;
    _odom_msg.twist.twist.angular.x = 0;
    _odom_msg.twist.twist.angular.y = 0;
    _odom_msg.twist.twist.angular.z = pos[2];

    _pubOdometry.publish(_odom_msg);
}

void
QuinticWalkingNode::publishMarker(std::string name_space, std::string frame, geometry_msgs::Pose pose, float r, float g,
                                  float b, float a) {
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = frame;

    marker_msg.type = marker_msg.ARROW;
    marker_msg.ns = name_space;
    marker_msg.action = marker_msg.ADD;
    marker_msg.pose = pose;


    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    marker_msg.color = color;

    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.003;
    scale.z = 0.003;
    marker_msg.scale = scale;

    marker_msg.id = _marker_id;
    _marker_id++;

    _pubDebugMarker.publish(marker_msg);
}

void QuinticWalkingNode::publishMarkers() {
    //publish markers
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    if (_walkEngine.getFootstep().isLeftSupport()) {
        marker_msg.header.frame_id = "l_sole";
    } else {
        marker_msg.header.frame_id = "r_sole";
    }
    marker_msg.type = marker_msg.CUBE;
    marker_msg.action = 0;
    marker_msg.lifetime = ros::Duration(0.0);
    geometry_msgs::Vector3 scale;
    scale.x = 0.20;
    scale.y = 0.10;
    scale.z = 0.01;
    marker_msg.scale = scale;
    //last step
    marker_msg.ns = "last_step";
    marker_msg.id = 1;
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker_msg.color = color;
    geometry_msgs::Pose pose;
    Eigen::Vector3d step_pos = _walkEngine.getFootstep().getLast();
    geometry_msgs::Point point;
    point.x = step_pos[0];
    point.y = step_pos[1];
    point.z = 0;
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);

    //last step center
    marker_msg.ns = "step_center";
    marker_msg.id = _marker_id;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;
    marker_msg.scale = scale;
    _pubDebugMarker.publish(marker_msg);

    // next step
    marker_msg.id = _marker_id;
    marker_msg.ns = "next_step";
    scale.x = 0.20;
    scale.y = 0.10;
    scale.z = 0.01;
    marker_msg.scale = scale;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 0.5;
    marker_msg.color = color;
    step_pos = _walkEngine.getFootstep().getNext();
    point.x = step_pos[0];
    point.y = step_pos[1];
    pose.position = point;
    pose.orientation = tf::createQuaternionMsgFromYaw(step_pos[2]);
    marker_msg.pose = pose;
    _pubDebugMarker.publish(marker_msg);

    _marker_id++;
}

void QuinticWalkingNode::initializeEngine() {
    _walkEngine.reset();
}

bool QuinticWalkingNode::SetHeadMoveValid(std_srvs::SetBool::Request& req,
                                          std_srvs::SetBool::Response& res) {
  std::cout << "Call Service SetHeadMoveValid: " << req.data << std::endl;
  _dsp_handler->SetHeadMoveValid(req.data);
  can_head_move = req.data;
  res.success = true;
  std::cout << "Call Service SetHeadMoveValid Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::SetSensorEnableValid(
    std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  std::cout << "Call Service SetSensorEnableValid: " << req.data << std::endl;
  _dsp_handler->SetSensorEnableValid(req.data);
  res.success = true;
  std::cout << "Call Service SetSensorEnableValid Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::ResetOdometry(std_srvs::SetBool::Request& req,
                                       std_srvs::SetBool::Response& res) {
  std::cout << "Call Service ResetOdometry: " << req.data << std::endl;
  _dsp_handler->ResetOdometry(req.data);
  _odometry_reset_pending = true;
  res.success = true;
  std::cout << "Call Service ResetOdometry Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::SetSpecialGaitValid(bitbots_msgs::SpecialGait::Request& req, bitbots_msgs::SpecialGait::Response& res) {
  std::cout << "Call Service SetSpecialGaitValid: " << req.data << std::endl;
  try
  {
  	_dsp_handler->SetSpecialGaitValid(req.data);

	  switch (req.gaitId){
		case StandUp:
		  _dsp_handler->SetSpecialGaitId(StandUp, 1);
		  res.success = true;
		  return true;
		case kDoLeftKick:
		  _dsp_handler->SetSpecialGaitId(kDoLeftKick, 1);
		  res.success = true;
		  return true;
		case kDoRightKick:
		  _dsp_handler->SetSpecialGaitId(kDoRightKick, 1);
		  res.success = true;
		  return true;
		case kDoStandFront:
		  _dsp_handler->SetSpecialGaitId(kDoStandFront, 1);
		  res.success = true;
		  return true;
		case kDoStandBack:
		  _dsp_handler->SetSpecialGaitId(kDoStandBack, 1);
		  res.success = true;
		  return true;
		case kDoLeftSave:
		  _dsp_handler->SetSpecialGaitId(kDoLeftSave, 1);
		  res.success = true;
		  return true;
		case kDoRightSave:
		  _dsp_handler->SetSpecialGaitId(kDoRightSave, 1);
		  res.success = true;
		  return true;
	  }
  }catch (std::exception e){
	std::cout<< " No GaitId Matched! "<< std::endl;
  	return false;
  } 
}

bool QuinticWalkingNode::SetTorqueEnable(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& res) {
  std::cout << "Call Service SetTorqueEnable: " << req.data << std::endl;
  _dsp_handler->SetTorqueEnable(req.data);
  res.success = true;
  std::cout << "Call Service SetTorqueEnable Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::SetGaitValid(std_srvs::SetBool::Request& req,
                                      std_srvs::SetBool::Response& res) {
  std::cout << "Call Service SetGaitValid: " << req.data << std::endl;
  _dsp_handler->SetGaitValid(req.data);
  res.success = true;
  std::cout << "Call Service SetGaitValid Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::ResetGait(std_srvs::SetBool::Request& req,
                                   std_srvs::SetBool::Response& res) {
  std::cout << "Call Service ResetGait: " << req.data << std::endl;
  _dsp_handler->ResetGait(req.data);
  res.success = true;
  std::cout << "Call Service ResetGait Done" << std::endl;
  return true;
}

bool QuinticWalkingNode::GetSpecialGaitPending(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::cout << "Call Service GetSpecialGaitPending" << std::endl;
  res.success = _special_gait_pending;
  std::cout << "Call Service GetSpecialGaitPending Done " << res.success
            << std::endl;
  return true;
}

bool QuinticWalkingNode::GetOdometryResetPending(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::cout << "Call Service GetOdometryResetPending" << std::endl;
  res.success = _odometry_reset_pending;
  std::cout << "Call Service GetOdometryResetPending Done " << res.success
            << std::endl;
  return true;
}

bool QuinticWalkingNode::GaitResetPending(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::cout << "Call Service GaitResetPending" << std::endl;
  res.success = _gait_reset_pending;
  std::cout << "Call Service GaitResetPending Done " << res.success
            << std::endl;
  return true;
}

bool QuinticWalkingNode::GetWalkKickPending(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::cout << "Call Service GetWalkKickPending" << std::endl;
  res.success = _walk_kick_pending;
  std::cout << "Call Service GetWalkKickPending Done " << res.success
            << std::endl;
  return true;
}

void QuinticWalkingNode::SetDspCommand() {
 if(first_stand)
 {
   std::cout << "first_stand" << first_stand << std::endl;
   _dsp_handler->SetHeadMoveValid(true);
   _headPos[0]=0;
   _headPos[1]=0;
   _dsp_handler->SetSpecialGaitValid(true);//SSSSS
   _dsp_hanlder->SetSpecialGaitId(init_stand,1);
   ros::Duration(5).sleep();
 }
  _dsp_handler->SetVelocity(_currentOrders[0], _currentOrders[1],
                            _currentOrders[2]);
  _dsp_handler->SetHeadPos(_headPos[0], _headPos[1]);
  //_dsp_handler->SetSpecialGaitId(_gaitId[0], _gaitId[1]);
}

void QuinticWalkingNode::ClearDspValidState() {
  _dsp_handler->SetSpecialGaitValid(false);
  _dsp_handler->ResetGait(false);
  _dsp_handler->ResetOdometry(false);
  _dsp_handler->DoWalkKickLeft(false);
  _dsp_handler->DoWalkKickRight(false);
}

void QuinticWalkingNode::GetDataFromDsp() {
  std::vector<bool> pending_states = _dsp_handler->GetSpecialGaitState();
  _special_gait_pending = pending_states[0];
  _odometry_reset_pending = pending_states[1];
  _gait_reset_pending = pending_states[2];
  _walk_kick_pending = pending_states[3];

  _real_velocities = _dsp_handler->GetVelocity();
  // std::cout << "real velocity: " << _real_velocities.transpose() << std::endl;

  _real_head_pos = _dsp_handler->GetHeadPos();

  _real_odometry = _dsp_handler->GetOdometry();

  _real_body_pos = _dsp_handler->GetBodyPose();

  _imu_angle = _dsp_handler->GetImuAngle(); // in order of roll, pitch, yaw
  //std::cout << "Odometry is: " << _imu_angle[0] << " " << _imu_angle[1] << " " << _imu_angle[2] << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "quintic_walking");
    // init node
    QuinticWalkingNode node = QuinticWalkingNode();
    // set the dynamic reconfigure and load standard params
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig> server;
    dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>::CallbackType f;
    f = boost::bind(&QuinticWalkingNode::reconf_callback, &node, _1, _2);
    server.setCallback(f);

    // run the node
    node.initializeEngine();
    node.run();
}
