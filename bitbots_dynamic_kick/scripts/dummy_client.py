#!/usr/bin/env python

from __future__ import print_function
import sys

from time import sleep
import random
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback

showing_feedback = False

if __name__ == "__main__":
    print("[..] Initializing node", end='')
    rospy.init_node('dynamic_kick_dummy_client', anonymous=True)
    print("\r[OK] Initializing node")


    def done_cb(state, result):
        print('Action completed: ', end='')
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)
        print(str(result))


    def active_cb():
        print("Server accepted action")


    def feedback_cb(feedback):
        if len(sys.argv) > 1 and sys.argv[1] == '--feedback':
            print('Feedback')
            print(feedback)
            print()


    print('[..] Connecting to action server \'dynamic_kick\'', end='')
    sys.stdout.flush()
    client = actionlib.SimpleActionClient('dynamic_kick', KickAction)
    if not client.wait_for_server():
        exit(1)
    print('\r[OK] Connecting to action server \'dynamic_kick\'')
    print()

    goal = KickGoal()
    goal.foot_target.header.stamp = rospy.Time.now()
    goal.foot_target.header.frame_id = 'base_footprint'
    goal.foot_target.pose.position.x = 0.1
    goal.foot_target.pose.position.y = -0.2
    goal.foot_target.pose.position.z = 0.15
    goal.foot_target.pose.orientation.w = 1
    goal.foot_speed = random.randrange(1000, 5000) / 1000  # random between 1.0 and 5.0
    client.send_goal(goal)
    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    print("Sent new goal. Waiting for result")
    client.wait_for_result()
