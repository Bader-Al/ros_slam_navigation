
import rospy, time
from geometry_msgs.msg import Point 
from std_msgs.msg import String

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped 

## TODO : switch to AMCL instead of odometery



class BotSubscriber: 

    def __init__(self, ):
        self.currentBotLocation = Point()
        self.listen_for_pose()
        
        

    def get_current_location(self):
        self.listen_for_pose()
        time.sleep(2) ## TODO :: Make it wait properly
        return self.currentBotLocation
    
        

    def listen_for_pose(self):
        # rospy.Subscriber("odom", Odometry, self.updateBotLocation_ODOM)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updateBotLocation_AMCL)
        # rospy.spin()


    def updateBotLocation_ODOM(self,odomMsg):
        self.currentBotLocation
        self.currentBotLocation.x = odomMsg.pose.pose.position.x
        self.currentBotLocation.y = odomMsg.pose.pose.position.y
        self.currentBotLocation.z = odomMsg.pose.pose.position.z

    def updateBotLocation_AMCL(self,AMCL_POSE):
        self.currentBotLocation
        self.currentBotLocation.x = AMCL_POSE.pose.pose.position.x
        self.currentBotLocation.y = AMCL_POSE.pose.pose.position.y
        self.currentBotLocation.z = AMCL_POSE.pose.pose.position.z

if __name__ =='__main__' :
    botSubscriber = BotSubscriber()
    rospy.init_node('bot_subscriber', anonymous=True)
    print('Runing bot subscribber test')
    botSubscriber.listen_for_pose()
    location = botSubscriber.get_current_location()
    while True: 
        print('coordinates' , location)
    