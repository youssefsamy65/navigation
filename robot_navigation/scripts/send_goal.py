#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x):
    try:
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
    
        if x == 1:
            goal.target_pose.pose.position.x = 0.27
            goal.target_pose.pose.position.y = 0.2
            goal.target_pose.pose.orientation.z = 0.1
            goal.target_pose.pose.orientation.w = 0.1
    
        elif x == 2:
            goal.target_pose.pose.position.x = 1.55
            goal.target_pose.pose.position.y = 2.12
            goal.target_pose.pose.orientation.z = -0.034
            goal.target_pose.pose.orientation.w = 0.99   
        else:
            goal.target_pose.pose.position.x = -0.102
            goal.target_pose.pose.position.y = 0.122
            goal.target_pose.pose.orientation.z = -0.028
            goal.target_pose.pose.orientation.w = 0.999
            
        client.send_goal(goal)
        wait = client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted.")
    except Exception as e:
        rospy.logerr("An error occurred: {}".format(e))

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client(1)
        rospy.loginfo("1st Goal execution done!")
        rospy.sleep(5)

        rospy.loginfo("2nd Goal execution done!")
        rospy.sleep(5)
 
        if result:
            rospy.loginfo("Goal execution done! Result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

