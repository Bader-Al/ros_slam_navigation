#!/usr/bin/env python  
from bot_client import BotClient
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import String


action_demand = "" 
prev_goal = None



''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Coordinates and goals 

'''''''''''''''''''''''''''''''''''''''

def listen_for_goals():     
    rospy.Subscriber("goal_coordinates", Point, set_goal ) 


def set_goal(incoming_goal): 
    xGoal = incoming_goal.x
    yGoal = incoming_goal.y 
    
    BotClient.move_to_goal(xGoal, yGoal)
 
        
   

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Demands (STILL NOT IMPLEMENTED)

'''''''''''''''''''''''''''''''''''''''

def listen_for_demands():
    rospy.Subscriber("goal_demands", String, set_demand )
    
     
def set_demand(demand): ## Still not implemented..
    global action_demand  
    
    if not demand.data: print('Waiting for demand [STOP, MOVE, CLEAR, etc.. ] ' )
 
    if demand.data != action_demand:
        print('Setting new demand to', demand.data) 
        action_demand = demand.data
   
    if demand.data == "STOP": ## This may conflict with the DRY principle but we did this for safety [incase global action_demand wasn't updated]    
        stop_movement()    
   
    if action_demand == "CLEAR":
        BotClient.ac.cancel_all_goals() 
        print("Clearing..") 
    
def stop_movement():
    print('CANCELLING ALL GOALS') 
    BotClient.ac.cancel_all_goals()



   

if __name__ == '__main__':
  
   rospy.init_node('map_navigation', anonymous=False)
 
   print('start go to goal') 
   listen_for_goals() 
   listen_for_demands() 

   rospy.spin()     
       
   
   
   

