## bitbots quintic walk
### What this repository for?
1. Subscribes to topic /cmd_vel for velocity control
2. Subscribes to topic /head_pos for head pitch/yaw control
3. Provides services for special gait control

### Usage
**Subscribed Topics:**

***/cmd_vel [geometry_msgs::Twist msg]:*** msg.linear.x for speed x [m/s], msg.linear.y for speed y [m/s], msg.angular.z for angular speed z [rad/s]

***/head_pos [sensor_msgs::JointState msg]:*** msg.position[0] for pitch [rad], msg.position[1] for yaw [rad]

**Services:**

***/do_left_kick [std_srvs::Trigger]:*** call for do kick with left leg

***/do_right_kick [std_srvs::Trigger]:*** call for do kick with right leg

***/do_stand_back [std_srvs::Trigger]:*** call for do standup with face sky

***/do_stand_front [std_srvs::Trigger]:*** call for do standup with face ground

***/do_walk_kick_left [std_srvs::Trigger]:*** call for do walk kick with left leg

***/do_walk_kick_right [std_srvs::Trigger]:*** call for do walk kick with right leg
