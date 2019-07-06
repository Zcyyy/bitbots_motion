#include "bitbots_dynamic_kick/Stabilizer.h"
#include "bitbots_dynamic_kick/DynamicBalancingGoal.h"
#include "bitbots_dynamic_kick/ReferenceGoals.h"

Stabilizer::Stabilizer() :
        m_visualizer("/debug/dynamic_kick") {
    /* load MoveIt! model */
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));

    /* Extract joint groups from loaded model */
    m_kinematic_model = robot_model_loader.getModel();
    m_all_joints_group = m_kinematic_model->getJointModelGroup("All");
    m_legs_joints_group = m_kinematic_model->getJointModelGroup("Legs");

    /* Reset kinematic goal to default */
    m_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
    m_goal_state->setToDefaultValues();

    reset();

    /* Initialize collision model */
    m_planning_scene.reset(new planning_scene::PlanningScene(m_kinematic_model));
}

void Stabilizer::reset() {
    /* We have to set some good initial position in the goal state,
     * since we are using a gradient based method. Otherwise, the
     * first step will be not correct */
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++) {
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }
    m_cop_x_error_sum = 0.0;
    m_cop_y_error_sum = 0.0;
}

std::optional<JointGoals> Stabilizer::stabilize(bool is_left_kick, geometry_msgs::Pose trunk_pose,
        geometry_msgs::Pose flying_foot_goal_pose, bool cop_support_point) {
    /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;
    double bio_ik_timeout = 0.01;

    tf::Transform trunk_goal;
    if (cop_support_point && m_use_cop) {
        /* calculate stabilizing target from center of pressure
         * the cop is in corresponding sole frame
         * optimal stabilizing would be centered above sole center */
        double cop_x, cop_y, last_cop_x, last_cop_y, cop_x_error, cop_y_error;
        if (is_left_kick) {
            cop_x = m_cop_right.x;
            cop_y = m_cop_right.y;
        } else {
            cop_x = m_cop_left.x;
            cop_y = m_cop_left.y;
        }
        cop_x_error = cop_x - trunk_pose.position.x;
        cop_y_error = cop_y - trunk_pose.position.y;
        m_cop_x_error_sum += cop_x_error;
        m_cop_y_error_sum += cop_y_error;
        double x = trunk_pose.position.x - cop_x * m_p_x_factor - m_i_x_factor * m_cop_x_error_sum - m_d_x_factor * (cop_x_error - m_cop_x_error);
        double y = trunk_pose.position.y - cop_y * m_p_y_factor - m_i_y_factor * m_cop_y_error_sum - m_d_y_factor * (cop_y_error - m_cop_y_error);
        m_cop_x_error = cop_x_error;
        m_cop_y_error = cop_y_error;

        /* Do not use control for height and rotation */
        trunk_goal.setOrigin({x, y, trunk_pose.position.z});

        /* Display projection of trunk goal on ground */
        m_visualizer.display_stabilizing_point({x, y, 0}, is_left_kick ? "r_sole" : "l_sole");
    } else {
        trunk_goal.setOrigin({trunk_pose.position.x, trunk_pose.position.y, trunk_pose.position.z});
    }
    trunk_goal.setRotation({trunk_pose.orientation.x, trunk_pose.orientation.y,
                            trunk_pose.orientation.z, trunk_pose.orientation.w});

    auto *bio_ik_trunk_goal = new ReferencePoseGoal();
    bio_ik_trunk_goal->setPosition(trunk_goal.getOrigin());
    bio_ik_trunk_goal->setOrientation(trunk_goal.getRotation());
    bio_ik_trunk_goal->setLinkName("base_link");
    if (is_left_kick) {
        bio_ik_trunk_goal->setReferenceLinkName("r_sole");
    } else {
        bio_ik_trunk_goal->setReferenceLinkName("l_sole");
    }
    ik_options.goals.emplace_back(bio_ik_trunk_goal);

    tf::Transform flying_foot_goal;
    flying_foot_goal.setOrigin({flying_foot_goal_pose.position.x,
                                flying_foot_goal_pose.position.y,
                                flying_foot_goal_pose.position.z});
    flying_foot_goal.setRotation({flying_foot_goal_pose.orientation.x,
                                  flying_foot_goal_pose.orientation.y,
                                  flying_foot_goal_pose.orientation.z,
                                  flying_foot_goal_pose.orientation.w});


    /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
    auto *bio_ik_flying_foot_goal = new ReferencePoseGoal();
    bio_ik_flying_foot_goal->setPosition(flying_foot_goal.getOrigin());
    bio_ik_flying_foot_goal->setOrientation(flying_foot_goal.getRotation());
    if (is_left_kick) {
        bio_ik_flying_foot_goal->setLinkName("l_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("r_sole");
    } else {
        bio_ik_flying_foot_goal->setLinkName("r_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("l_sole");
    }
    bio_ik_flying_foot_goal->setWeight(m_flying_weight);

    ik_options.goals.emplace_back(bio_ik_flying_foot_goal);

    bool success = m_goal_state->setFromIK(m_legs_joints_group,
                                           EigenSTL::vector_Isometry3d(),
                                           std::vector<std::string>(),
                                           bio_ik_timeout,
                                           moveit::core::GroupStateValidityCallbackFn(),
                                           ik_options);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = m_planning_scene->getAllowedCollisionMatrix();
    m_planning_scene->checkCollision(req, res, *m_goal_state, acm);
    if (res.collision) {
        ROS_ERROR_STREAM("Aborting due to self collision!");
        //success = false;
    }

    if (success) {
        /* retrieve joint names and associated positions from  */
        std::vector<std::string> joint_names = m_legs_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        m_goal_state->copyJointGroupPositions(m_legs_joints_group, joint_goals);

        /* construct result object */
        JointGoals result;
        result.first = joint_names;
        result.second = joint_goals;
        return result;
    } else {
        return std::nullopt;
    }
}

void Stabilizer::use_stabilizing(bool use) {
    m_use_stabilizing = use;
}

void Stabilizer::use_minimal_displacement(bool use) {
    m_use_minimal_displacement = use;
}

void Stabilizer::use_cop(bool use) {
    m_use_cop = use;
}

void Stabilizer::set_trunk_height(double height) {
    m_trunk_height = height;
}

void Stabilizer::set_stabilizing_weight(double weight) {
    m_stabilizing_weight = weight;
}

void Stabilizer::set_flying_weight(double weight) {
    m_flying_weight = weight;
}

void Stabilizer::set_trunk_orientation_weight(double weight) {
    m_trunk_orientation_weight = weight;
}

void Stabilizer::set_trunk_height_weight(double weight) {
    m_trunk_height_weight = weight;
}

void Stabilizer::set_p_factor(double factor_x, double factor_y) {
    m_p_x_factor = factor_x;
    m_p_y_factor = factor_y;
}

void Stabilizer::set_i_factor(double factor_x, double factor_y) {
    m_i_x_factor = factor_x;
    m_i_y_factor = factor_y;
}

void Stabilizer::set_d_factor(double factor_x, double factor_y) {
    m_d_x_factor = factor_x;
    m_d_y_factor = factor_y;
}
