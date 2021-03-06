#include "bitbots_dynup/DynupNode.h"

DynUpNode::DynUpNode() :
        m_server(m_node_handle, "dynup", boost::bind(&DynUpNode::execute_cb, this, _1), false),
        m_listener(m_tf_buffer) {
    m_joint_goal_publisher = m_node_handle.advertise<bitbots_msgs::JointCommand>("animation_motor_goals", 1);
    m_server.start();
}

void DynUpNode::reconfigure_callback(bitbots_dynup::DynUpConfig &config, uint32_t level) {
    m_engine_rate = config.engine_rate;

    DynUpParams params = DynUpParams();
    //TODO actually set parameters like
    //params.leg_min_length = config.leg_min_length;
    params.foot_distance = config.foot_distance;
    params.rise_time = config.rise_time;
    params.trunk_x = config.trunk_x;
    params.trunk_height = config.trunk_height;
    params.trunk_pitch = config.trunk_pitch;

    m_engine.set_params(params);

    m_engine.m_stabilizer.use_minimal_displacement(config.minimal_displacement);
    m_engine.m_stabilizer.use_stabilizing(config.stabilizing);
    m_engine.m_stabilizer.set_stabilizing_weight(config.stabilizing_weight);
    m_engine.m_stabilizer.set_flying_weight(config.flying_weight);
    m_engine.m_stabilizer.set_trunk_orientation_weight(config.trunk_orientation_weight);
}

void DynUpNode::execute_cb(const bitbots_msgs::DynUpGoalConstPtr &goal) {
    // TODO: maybe switch to goal callback to be able to reject goals properly
    ROS_INFO("Accepted new goal");
    m_engine.reset();

    if (std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> poses = get_current_poses()) {
        m_engine.start(true, poses->first, poses->second); //todo we are currently only getting up from squad
        loop_engine();
        bitbots_msgs::DynUpResult r;
        r.successful = true;
        m_server.setSucceeded(r);
    } else {
        ROS_ERROR_STREAM("Could not determine foot positions! Aborting standup.");
        bitbots_msgs::DynUpResult r;
        r.successful = false;
        m_server.setAborted(r);
    }
}

void DynUpNode::loop_engine() {
    /* Do the loop as long as nothing cancels it */
    while (m_server.isActive() && !m_server.isPreemptRequested()) {
        if (std::optional<JointGoals> goals = m_engine.tick(1.0 / m_engine_rate)) {
            // TODO: add counter for failed ticks
            bitbots_msgs::DynUpFeedback feedback;
            feedback.percent_done = m_engine.get_percent_done();
            m_server.publishFeedback(feedback);
            publish_goals(goals.value());

            if (feedback.percent_done == 100) {
                break;
            }
        }

        /* Let ROS do some important work of its own and sleep afterwards */
        ros::spinOnce();
        ros::Rate loop_rate(m_engine_rate);
        loop_rate.sleep();
    }
}

std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> DynUpNode::get_current_poses() {
    // return left foot pose and trunk pose relative to right foot
    ros::Time time = ros::Time::now();

    /* Construct zero-positions for both poses in their respective local frames */
    geometry_msgs::PoseStamped l_foot_origin, trunk_origin;
    l_foot_origin.header.frame_id = "l_sole";
    l_foot_origin.pose.orientation.w = 1;
    l_foot_origin.header.stamp = time;

    trunk_origin.header.frame_id = "torso";
    trunk_origin.pose.orientation.w = 1;
    trunk_origin.header.stamp = time;

    /* Transform both poses into the right foot frame */
    geometry_msgs::PoseStamped l_foot_transformed, trunk_transformed;
    try {
        m_tf_buffer.transform(l_foot_origin, l_foot_transformed, "r_sole",
                              ros::Duration(0.2)); // TODO lookup thrown exceptions in internet and catch
        m_tf_buffer.transform(trunk_origin, trunk_transformed, "r_sole", ros::Duration(0.2));
        return std::pair(l_foot_transformed.pose, trunk_transformed.pose);
    } catch (tf2::TransformException) {
        return std::nullopt;
    }

}

void DynUpNode::publish_goals(const JointGoals &goals) {
    /* Construct JointCommand message */
    bitbots_msgs::JointCommand command;
    command.header.stamp = ros::Time::now();

    /*
     * Since our JointGoals type is a vector of strings
     *  combined with a vector of numbers (motor name -> target position)
     *  and bitbots_msgs::JointCommand needs both vectors as well,
     *  we can just assign them
     */
    command.joint_names = goals.first;
    command.positions = goals.second;

    /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
    std::vector<double> vels(goals.first.size(), -1.0);
    std::vector<double> accs(goals.first.size(), -1.0);
    std::vector<double> pwms(goals.first.size(), -1.0);
    command.velocities = vels;
    command.accelerations = accs;
    command.max_currents = pwms;

    m_joint_goal_publisher.publish(command);
}

int main(int argc, char *argv[]) {
    /* Setup ROS node */
    ros::init(argc, argv, "dynup");
    DynUpNode node;

    /* Setup dynamic_reconfigure */
    dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig> dyn_reconf_server;
    dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig>::CallbackType f;
    f = boost::bind(&DynUpNode::reconfigure_callback, &node, _1, _2);
    dyn_reconf_server.setCallback(f);

    ROS_INFO("Initialized DynUp and waiting for actions");
    ros::spin();
}

