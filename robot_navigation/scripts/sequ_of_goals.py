#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def move_to_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

def get_speech(data, move_base_client):
    speech_text = data.data.lower()  # Convert to lowercase for case insensitivity
    if "move" in speech_text or "go" in speech_text or "come to me" in speech_text:
        pose = Pose()
        pose.position.x = 0.27
        pose.position.y = 0.2
        pose.orientation.z = 0.1
        pose.orientation.w = 0.1
        move_base_client.move_to_pose(pose)

def listener(move_base_client):
    rospy.loginfo("Speech Recognition")
    rospy.Subscriber("/chatter", String, get_speech, move_base_client)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_move')
        move_base_client = MoveBaseClient()
        listener(move_base_client)
    except rospy.ROSInterruptException:
        pass

