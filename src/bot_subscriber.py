
import rospy, time
from geometry_msgs.msg import Point 
from std_msgs.msg import String

from nav_msgs.msg import Odometry





class BotSubscriber: 

    def __init__(self, ):
        self.currentBotLocation = Point()
        self.listen_for_pose()
        
        


    def get_current_location(self):
        self.listen_for_pose()
        time.sleep(2) ## TODO :: Make it wait properly
        return self.currentBotLocation

    def listen_for_pose(self):
        rospy.Subscriber("odom", Odometry, self.updateBotLocation)
        # rospy.spin()


    def updateBotLocation(self,odomMsg):
        self.currentBotLocation
        self.currentBotLocation.x = odomMsg.pose.pose.position.x
        self.currentBotLocation.y = odomMsg.pose.pose.position.y
        self.currentBotLocation.z = odomMsg.pose.pose.position.z

        # print('now bot location X:', self.currentBotLocation.x)
        # print('now bot location Y:', self.currentBotLocation.y)

