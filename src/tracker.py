import rospy
import cv_bridge
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus


def pursuer_position_callback(msg):




if __name__ == '__main__':
    rospy.init_node('pursuer_control_node',anonymous=True)
    rospy.Subscriber('/tb3_0/amcl_pose',PoseWithCovarianceStamped,pursuer_position_callback,queue_size=10)
    pursuer_goal = rospy.Publisher('/tb3_0/move_base_simple/goal',PoseStamped,queue_size=10)
    pursuer_goal_status = rospy.Publisher('/tb3_0/move_base_simple/goal',GoalStatus)
    