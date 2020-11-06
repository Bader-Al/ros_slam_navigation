#!/usr/bin/env python  
import math
import os, sys
from time import sleep
from typing import OrderedDict
from actionlib import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


from user_interface import UserInterface
from locations.locations_parser import LocationsParser
from utils import CoordinatesUtils

from bot_subscriber import BotSubscriber



import rospy, time
from geometry_msgs.msg import Point 
from std_msgs.msg import String

 
userInterface = None
botSubscriber = None 
parser = LocationsParser()


coordPublisher = rospy.Publisher('goal_coordinates', Point, queue_size=10)  ## Sends Coordinates as [X,Y,Z] ... Example :: [0.232342 , 0.454532 , 0 ]
commandPublisher = rospy.Publisher('goal_demands', String, queue_size= 10) ## Sends Demands as "STRING" ... Example :: "STOP" | "MOVE" | "CANCEL"




def cancel_and_getGoal():
    send_clear()
    return get_UserGoal( skipWelcome=True)



def get_UserGoal(skipWelcome = False):
    global userInterface

    ## The way it reinits the user interface depends on wether it was initialized before (To skip welcome)
    if (skipWelcome): userInterface = UserInterface(skipWelcome=True)
    else: userInterface = UserInterface(skipWelcome=True)

    userGoal=None
    userInterfaceOutput = iter(userInterface)
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
         sortedGoalsDictList = _getSortedGoalCoordinates(goalSetOfNames=goal) 
         publish_tour_goals(sortedGoalsDictList)

    else: print("something went wrong..",goal)


def _getSortedGoalCoordinates(goalSetOfNames):
    unsortedCoordinatesDictList = []
    for goalName in goalSetOfNames:
        tempDict = {}
        XY_coordinates = parser.get_XY_Coordinates(goalName)
        tempDict.update({'name' : goalName ,"coordinates": XY_coordinates, } )
        unsortedCoordinatesDictList.append(tempDict)

    botLocation = botSubscriber.get_current_location()     
    orderedDictList = CoordinatesUtils.getCoordinates_sortedByDistance( unsortedCoordinatesDictList,botLocation, parser )
    # print(orderedDictList)
    return orderedDictList




def publish_tour_goals(sortedGoalsDictList):
    print("Starting tour!")

    if(len(sortedGoalsDictList) < 1):
        print("Tour has ended!")
        return get_UserGoal(skipWelcome=True)

    goal = sortedGoalsDictList.pop(0)
    _goalName = goal['name']
    _goalCoords = goal['coordinates']
    _goalDistance = goal['distance']
    # if 'toilet' in _goalName:continue ## Skips toilet in tours
    publish_goal(_goalName,_goalCoords, tour_mode=True)
    print ("Next goal coming up")
    botLocation = botSubscriber.get_current_location() 
    newlyOrderedDictList = CoordinatesUtils.getCoordinates_sortedByDistance( sortedGoalsDictList,botLocation, parser )
    print(newlyOrderedDictList) 
    return publish_tour_goals(newlyOrderedDictList)


       



def publish_goal(goalName,goalCoordinates, tour_mode= False): 
    print('publish function') 
    while not rospy.is_shutdown(): 
        xGoal, yGoal = goalCoordinates

        send_move()
        print('publishing', goalName)
        coordPublisher.publish(Point(xGoal,yGoal,0)) 
        
        if not tour_mode:
          action = userInterface.present_actionMenu(locationName=goalName, locationCoords = goalCoordinates )
          if action == '1' : return get_UserGoal( skipWelcome=True)  
          elif action == '2' : return cancel_and_getGoal()  # TODO :: Should allow user to input new goal after stopping bot!!
          elif action == '0' : emergency_stop()   # TODO :: Still not implemented properly

        else:
          botLocation = botSubscriber.get_current_location()
          deltaX = abs(botLocation.x - xGoal)
          deltaY = abs(botLocation.y - yGoal)
          threshold = 0.08
          
          if( deltaX*deltaY < threshold ):
           print("Arrived!")

           return True  
       
        

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

def send_clear():
    for i in range(25):commandPublisher.publish("CLEAR")
 

def send_move():
    commandPublisher.publish("MOVE")
 


if __name__ == "__main__":
    
    rospy.init_node('goals_publisher', anonymous=True)
    botSubscriber = BotSubscriber()
    get_UserGoal()

    
    
    

        

    
    