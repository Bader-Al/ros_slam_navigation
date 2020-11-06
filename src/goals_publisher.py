#!/usr/bin/env python  
import math
import os, sys
from time import sleep
from typing import OrderedDict
import actionlib



PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


from user_interface import UserInterface
from locations.locations_parser import LocationsParser
from utils import CoordinatesUtils

from bot_subscriber import BotSubscriber

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import rospy, time
from geometry_msgs.msg import Point 
from std_msgs.msg import String

 
userInterface = None
parser = LocationsParser()


coordPublisher = rospy.Publisher('goal_coordinates', Point, queue_size=10)  ## Sends Coordinates as [X,Y,Z] ... Example :: [0.232342 , 0.454532 , 0 ]
commandPublisher = rospy.Publisher('goal_demands', String, queue_size= 10) ## Sends Demands as "STRING" ... Example :: "STOP" | "MOVE" | "CANCEL"




def cancel_and_getGoal():
    send_cancel()
    get_UserGoal( skipWelcome=True)

def override_userGoal():
    """
        Sends override signal which tells subscriber to clear all goals 
        from bot but still continue listening for demands... 
        Expecting next demand to be MOVE 
    """
    send_override() 
    get_UserGoal( skipWelcome=True)



def get_UserGoal(skipWelcome = False):
    global userInterface

    ## The way it reinits the user interface depends on wether it was initialized before (To skip welcome)
    if (skipWelcome): userInterface = UserInterface(skipWelcome=True)
    else: userInterface = UserInterface(skipWelcome=True)

    userInterfaceOutput = iter(userInterface)
    userGoal = next(userInterfaceOutput)


    while True:
        try: userGoal = next(userInterfaceOutput)
        except StopIteration: 
            dealWithUserGoal(goal=userGoal)
        time.sleep(1)


def dealWithUserGoal(goal):
    print('goal came in to be published', goal)
    if type(goal) == tuple : # If goal is a tuple it would contain ( GOAL_NAME  ,  [GOAL_X , GOAL_Y]  )
         goalName = goal[0]
         goalCoordinates = goal[1] 
         publish_goal(goalName = goalName, goalCoordinates=goalCoordinates)
    elif type(goal) == set : # If goal is a set it would contain the names of all goals in the given location {room_1, room_2, etc..}
         sortedGoalsDictList = getSortedGoalCoordinates(goalSetOfNames=goal) 
         for goal in sortedGoalsDictList:
             _goalName = goal['name']
             _goalCoords = goal['coordinates']
             if 'toilet' in _goalName:continue ## Skips toilet in tours
             print("\nYour tour will include" , _goalName)
             publish_goal(goalName=_goalName, goalCoordinates=_goalCoords, tourMode=True)
             time.sleep(5)
         print('\n\nTour locations sent to the bot.. Starting tour!')
             
             
         

    else: print("something went wrong..",goal)


def getSortedGoalCoordinates(goalSetOfNames):
    unsortedCoordinatesList = []
    for goalName in goalSetOfNames:
        XY_coordinates = parser.get_XY_Coordinates(goalName)
        unsortedCoordinatesList.append([ goalName, XY_coordinates])   

    botLocation = BotSubscriber().get_current_location()     
    orderedDictList = CoordinatesUtils.getCoordinates_sortedByDistance( unsortedCoordinatesList,botLocation, parser )
    print('sorted goals according to distance')
    print(orderedDictList)
    return orderedDictList



def publish_goal(goalName,goalCoordinates, tourMode = False): 
    print('publish function')
    t0 = rospy.get_time()
    while not rospy.is_shutdown(): 
        deltaTime = int(rospy.get_time() - t0)
        xGoal, yGoal = goalCoordinates
        if not tourMode:
            action = userInterface.present_actionMenu(locationName=goalName, locationCoords = goalCoordinates ) # 1 - Override .. 2 - Stop and go back 
            if action == '1' : override_userGoal() # Working properly. However tests are still necessary!
            if action == '2' : cancel_and_getGoal()  # TODO :: Should allow user to input new goal after stopping bot!!
            if action == '0' : emergency_stop()   # TODO :: Still not implemented properly

        # rate.sleep() 
        if deltaTime % 2 == 0: 
         print('publishing')    
         commandPublisher.publish("MOVE")
         coordPublisher.publish(Point(xGoal,yGoal,0)) 
         if tourMode:return
        
        """ rate.sleep() Doesn't work here due to the interface waiting for the user input to return an action..
            Therefore publishing is controlled by a factor of time 
        """

        

def emergency_stop():
    print('Emergency STOP & SHUTDOWN initiated\n\n')
    # TODO :: Must stop bot!
    send_stop()
    rospy.signal_shutdown("User shutdown")
    sys.exit()


def send_stop():
    for i in range(50):commandPublisher.publish("STOP")

def send_cancel():
    for i in range(25):commandPublisher.publish("CANCEL")

def send_override():
    for i in range(25):commandPublisher.publish("OVERRIDE")
 


if __name__ == "__main__":
    
    rospy.init_node('goals_publisher', anonymous=True)
    get_UserGoal()

    
    
    

        

    
    