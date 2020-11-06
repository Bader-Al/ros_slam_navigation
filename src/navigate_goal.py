#!/usr/bin/env python  
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import String


action_demand = ""
past_goal = Point()
goalsList = []

 #define a client for to send goal requests to the move_base server through a SimpleActionClient
ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)



def listen_for_demands():
    rospy.Subscriber("goal_demands", String, set_demand )
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
   

def set_demand(demand):
    global action_demand, ac     
    
    if not demand.data: print('Waiting for demand [STOP, MOVE, CLEAR, etc.. ] ' )
 
    if demand.data != action_demand:
        print('Setting new demand to', demand.data) 
        action_demand = demand.data
   
    if demand.data == "STOP": ## This may conflict with the DRY principle but we did this for safety [incase global action_demand wasn't updated]    
        stop_movement()    
   

    if action_demand == "MOVE":    
        listen_for_goals()
    if action_demand == "STOP":    
        stop_movement()
    



def listen_for_goals():     
    rospy.Subscriber("goal_coordinates", Point, set_goal )
    iterateThrough_goals()


def set_goal(incoming_goal): 
    global ac, past_goal
    # if the incoming goal isn't new.. simply break out of this function
    if past_goal == incoming_goal: return
    else: 
        goalsList.append(incoming_goal)
        past_goal = incoming_goal



def stop_movement():
    global ac
    print('CANCELLING ALL GOALS') 
    ac.cancel_all_goals()
    



def iterateThrough_goals(): 
   print('iterating thru goals') 
   if not goalsList : return

   goal = goalsList.pop(0)
   xGoal = goal.x
   yGoal = goal.y 
    
   arrived = move_to_goal(xGoal, yGoal)
   if(arrived) : print('yay') #goalsList.remove(goal)
   else: print('oh no') 




#this method will make the robot move to the goal location
def move_to_goal(xGoal,yGoal):
   global ac
   
   goal = MoveBaseGoal()
   
   # Set up the frame parameters
   goal.target_pose.header.frame_id = "map"
   goal.target_pose.header.stamp = rospy.Time.now()

   # Moving towards the goal */
   goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
   goal.target_pose.pose.orientation.x = 0.0
   goal.target_pose.pose.orientation.y = 0.0
   goal.target_pose.pose.orientation.z = 0.0
   goal.target_pose.pose.orientation.w = 1.0

   

   server_wait_duration = rospy.Duration.from_sec(5.0)
   while(not ac.wait_for_server(server_wait_duration)): rospy.loginfo("Waiting for the move_base action server to come up")

   print('Sending goal to bot' , xGoal, yGoal)
   ac.send_goal(goal)


   #wait for the action server to come up
   ac.wait_for_result(rospy.Duration(60))

   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("You have reached the destination")
           return True

   else:
           rospy.loginfo("The robot failed to reach the destination")
           return False


   

if __name__ == '__main__':
  
   rospy.init_node('map_navigation', anonymous=False)
 
   print('start go to goal') 

   listen_for_demands()    
       
   
   
   
