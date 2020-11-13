
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy, time
from geometry_msgs.msg import Point 
from std_msgs.msg import String

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped 




class BotClient: 
    
    
    # prev_goal = None
    
    
    currentBotLocation = Point()
    
        
    statusPublisher = rospy.Publisher('move_status', String, queue_size= 10) ##  
    
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
    @staticmethod
    def move_to_goal( xGoal,yGoal):
                       
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Moving towards the goal */
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        server_wait_duration = rospy.Duration.from_sec(5.0)
        while(not BotClient.ac.wait_for_server(server_wait_duration)): rospy.loginfo("Waiting for the move_base action server to come up")

        print('Sending goal to bot' , xGoal, yGoal)
        BotClient.ac.send_goal(goal)


        #wait for the action server to come up
        BotClient.ac.wait_for_result(rospy.Duration(60))
        action_state = BotClient.ac.get_state() 
        
        

        if(action_state ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("You have reached the destination")
           BotClient.publish_status(1) # OK
           return True
        elif(action_state ==  GoalStatus.REJECTED):
           rospy.loginfo("Bot action server rejected your goal!")
           BotClient.publish_status(0) # BUSY
           return True

        else:
           rospy.loginfo("The robot failed to reach the destination")
           BotClient.publish_status(2) # WAIT
           return False

    
    @staticmethod
    def get_current_location( ):
        BotClient.listen_for_pose()
        time.sleep(2) ## TODO :: Make it wait properly
        return BotClient.currentBotLocation
    
    
    
    @staticmethod
    def publish_status(status_int): 
        if status_int == 0: BotClient.statusPublisher.publish("BUSY")
        elif status_int == 1: BotClient.statusPublisher.publish("OK")
        elif status_int == 2: BotClient.statusPublisher.publish("WAIT")
            
        
    @staticmethod
    def listen_for_pose( ):
        # rospy.Subscriber("odom", Odometry, self.updateBotLocation_ODOM)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, BotClient.updateBotLocation_AMCL) 
        

    @staticmethod
    def updateBotLocation_ODOM(odomMsg):
        BotClient.currentBotLocation.x = odomMsg.pose.pose.position.x
        BotClient.currentBotLocation.y = odomMsg.pose.pose.position.y
        BotClient.currentBotLocation.z = odomMsg.pose.pose.position.z
        
    @staticmethod
    def updateBotLocation_AMCL(AMCL_POSE):
        # print('updating pose')
        BotClient.currentBotLocation.x = AMCL_POSE.pose.pose.position.x
        BotClient.currentBotLocation.y = AMCL_POSE.pose.pose.position.y
        BotClient.currentBotLocation.z = AMCL_POSE.pose.pose.position.z
        
        

if __name__ =='__main__' :
    
    ## TEST METHOD
    
    rospy.init_node('bot_subscriber', anonymous=True)
    # botClient = BotClient()
    BotClient.move_to_goal(0.8803472795836789,3.07251281622427)
    print('Runing bot subscribber test')
    BotClient.listen_for_pose()
    location = BotClient.get_current_location()
    
    
    while not rospy.is_shutdown(): 
        print('\n\n\n\n')
        print('coordinates' , location)
        print('state' , BotClient.get_bot_state())
        
     
        
    