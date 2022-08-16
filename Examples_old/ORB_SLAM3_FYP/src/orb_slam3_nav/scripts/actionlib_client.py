#! /usr/bin/env python
import imp
import roslib
roslib.load_manifest('ORB_SLAM3')
import rospy
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped


class ActionLibClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

        rospy.loginfo("Waiting for server...")
        if not self.client.wait_for_server(): 
            rospy.logerr("Sever not available!")
        else:
            rospy.loginfo("Found server!")
            
        self.goal_sub = rospy.Subscriber("goal", PoseStamped, self.goal_cb )
        self.waiting = False
        pass

    def goal_cb(self, msg ):
        rospy.loginfo("Received goal: %s ", msg.pose)
        if(not self.waiting):
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose = msg
            self.client.send_goal(mb_goal, self.results_cb, self.active_cb, self.feedback_cb)
        else:
            mb_cancel = GoalID()
            mb_cancel.stamp = rospy.Time()
            self.cancel_pub.publish(mb_cancel)

    def active_cb(self, ):
        self.waiting = True
    def feedback_cb(self, feedback):pass
    def results_cb(self, state, result):
        rospy.loginfo("State: %s, result %s", state, result)
        self.waiting = False

if __name__ == '__main__':
    rospy.init_node('orb_slam3_nav_client')
    actionlib_client = ActionLibClient()

    rospy.spin()

    rospy.loginfo("Shut down...")

