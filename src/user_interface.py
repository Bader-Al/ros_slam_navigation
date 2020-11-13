#!/usr/bin/env python  
import enum, select, os, sys, rospy
from bot_client import BotClient
from warnings import catch_warnings
import time
from geometry_msgs.msg import Point



if os.name == 'nt':
  import msvcrt
else:
  import tty, termios



PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from locations.locations_parser import LocationsParser 




class StartUpMenu_Selection(enum.Enum):
    Tour = 1
    Goal = 2
    SwitchUser = 9


DNE_GOAL_ERROR = "Goal does not exist in the database! Please make sure you select a name within the provided list"
INVALID_GOAL_ERROR = "Invalid name! please enter a location name within the locations listed above"
INVALID_SELECTION_ERROR = "Invalid Selection! please input a valid option from the list presented "

parser = LocationsParser()

class UserInterface:

    def __init__(self, skipWelcome = False,  ):
        self.nextGoal = [] # X Y Coordinates
        self.prevGoal = self.nextGoal[::]
        if not skipWelcome:  print("\nWelcome to PSU Tour Bot!\n")

        while True:
            menu_selection = self._present_startupMenu()

            if menu_selection == StartUpMenu_Selection.Goal : 
                goalName = self._present_setGoalMenu()
                self.nextGoal = ( goalName,parser.get_XY_Coordinates(goalName) )
                break

            elif menu_selection == StartUpMenu_Selection.Tour:
                self.nextGoal = parser.get_positionNames() 
                break

            elif menu_selection  == StartUpMenu_Selection.SwitchUser:
                self._present_AdminPanel()
                 

            

    def __iter__(self):
        while self.nextGoal!=self.prevGoal:
            yield self.nextGoal
            self.prevGoal = self.nextGoal

 

    def _present_setGoalMenu(self):
        goalName = ""
        print("\nPlease select one of the following goal Locations")
        locationNames = parser.get_positionNames()
        for name in locationNames:
            print(name)
        while True:
            userGoalInput = input('\nGoal Location: ')

            if len(userGoalInput) < 3: print(INVALID_GOAL_ERROR);continue

            matchedGoalInDatabase = parser.find_Location(userGoalInput)

            if type(matchedGoalInDatabase) is str:
                goalName = matchedGoalInDatabase
                break
            elif type(matchedGoalInDatabase) is set and len(matchedGoalInDatabase) > 1:
                print('\n\nDid you mean...\n')
                for suggestedGoalName in matchedGoalInDatabase:
                    print(suggestedGoalName)
                continue
            else: 
                print(DNE_GOAL_ERROR)
                
        print('selected goal name:', goalName)
        return goalName
        


    def present_actionMenu(self, locationName, locationCoords): 
        print("\n\n\nPublishing", locationName)
        print("with Coordinates: ")
        print(locationCoords)
        print('\n[1] - Add location to que\n[2] - Clear location and start over\n\n[0] - Emergency STOP & SHUTDOWN\n\n')
        key = self.__getKey()
        return key




    def _present_startupMenu(self):

        while True:
            print(" \n1 - Take a tour\n2 - Set goal location\n3 - View All Locations\n----------------------\n9 - Admin Panel\n0 - Exit\n\n")
            user_selection = str(input())
            user_selection = user_selection.strip().lower()

            tourIsSelected =('tour' in user_selection or 'take'  in user_selection or user_selection in "1")
            setGoalIsSelected =('goal'  in user_selection or 'set'  in user_selection or user_selection in "2")
            dumpLocationsIsSelected = ('locations'  in user_selection or 'all'  in user_selection or user_selection in "3")
            switchUserIsSelected = ('switch'  in user_selection or 'admin'  in user_selection or user_selection in "9")
            exitIsSelected = ('exit'  in user_selection or 'quit'  in user_selection or user_selection in "0")

            if tourIsSelected: return StartUpMenu_Selection.Tour
            elif setGoalIsSelected: return StartUpMenu_Selection.Goal
            elif switchUserIsSelected: return StartUpMenu_Selection.SwitchUser
            # elif dumpLocationsIsSelected: return StartUpMenu_Selection.Dump
            elif dumpLocationsIsSelected: parser.dump_positions()
            elif exitIsSelected: sys.exit()  ## NOT SAFE

            else: print(INVALID_GOAL_ERROR)


    def _present_AdminPanel(self) :
         
        print(''' \n\n\nWelcome to the Admin Panel\n
        \n[1] - Change file path [%s]
        
        [2] - Add location
        [3] - Delete location
        [4] - View all Locations\n \
        \n------------------------------------\
        \n[9] - Back to Main Menu\n ''' %parser.getRelativePath() )
        
        while True: 
            user_selection=''
            user_selection = self.__getKey()

            if user_selection == '1': self._Admin_present_SetFilePath() ## TODO : implement
            elif user_selection == '2': return self._Admin_present_AddLocations()
            elif user_selection == '3': return self._Admin_present_DeleteLocations()
            elif user_selection == '9': return 
            elif user_selection == '4': 
                parser.dump_positions()
                time.sleep(3)
                return self._present_AdminPanel()
            
 


    def _Admin_present_SetFilePath(self):
        print("\n\nPATH_SET Still not implemented")
        
        
        
    
    def _Admin_present_AddLocations(self):
        
        while True:
          #Handle location name  
          locationName = input('\nLocation name to be added:')
          print('\n')  
          if len(locationName.strip()) < 3:
              print("Location name too short! Please try again...")
              continue
          print("Finding similar location names to the one you provided..\n")
          lookupResult = parser.find_Location(locationName)
          if type(lookupResult) is set:
              print("%s matches some other records in the dictionary:" %locationName)
              for name in lookupResult: print(name)
              break
          elif type(lookupResult) is str:
              print ('\n\n---WARNING---\n')
              print("%s ALREADY EXISTS in the database! Cancel OR OVERWRITE the saved location.." %locationName)
              break
          else: 
              print("Ace! Your desired name %s is very unique! That means nothing else looks like it in the databse"  %locationName)
              break
 

        print("How would you like to provide the coordinates: \n[0] - Cancel\n[1] - Auto-Detect\n[2] - Manual input\n[9] - Change name")
        coordinates = Point()
        while True:
          #Handle methedology OR EXIT   
          userInput = self.__getKey()
          if(userInput == '0'):return self._present_AdminPanel() 
          
          elif(userInput == '1'): # Auto Coordinates
            coordinates = self._Admin_methods_LocationAutoInput(locationName)
            break

          elif(userInput == '2'): 
            coordinates = self._Admin_methods_LocationManualInput()
            break

          elif(userInput == '9'): 
            return self._Admin_present_AddLocations() # Recurse

          else:continue  


        parser.insert_Location(locationName, coordinates_point=coordinates )
        return self._present_AdminPanel()






        #   try: 
        #       parser.delete_Location(location_name=locationToBeDeleted)
        #       print("Successfully deleted", locationToBeDeleted)
        #   except: print("\n\n[-][!] Deletion failed! This typically means that the location DNE in the DB")
    

    def _Admin_methods_LocationManualInput(self):
        print("Please the coordinates:")
        X = None
        Y = None
        Z = 0

        while X == None:
         try: X = float(input("X:"))
         except: print("Please enter a valid coordinate!")

        while Y == None:
         try: Y = float(input("Y:"))
         except: print("Please enter a valid coordinate!")
        
        print("Z is not supported yet.. Skipping Z")
        time.sleep(1)
        coords = Point(X ,Y ,Z)
        return coords

    def _Admin_methods_LocationAutoInput(self, locationName):
        print("\nFetching bot coordinates..\n\n") 
        coords = BotClient.get_current_location()
        print('cords',coords)
        return coords

        



    def _Admin_present_DeleteLocations(self):
        while True:
          locationToBeDeleted = input("\n'0' - main menu\n'1' - admin menu\n\nLocation to be deleted:")
          if locationToBeDeleted == '0':return self._present_startupMenu()
          elif locationToBeDeleted == '1' : return self._present_AdminPanel()
          try: 
              parser.delete_Location(location_name=locationToBeDeleted)
              print("Successfully deleted", locationToBeDeleted)
          except: print("\n\n[-][!] Deletion failed! This typically means that the location DNE in the DB")
        # self._present_AdminPanel( )
         
    


# Utility method used to get raw input from keyboard instantly.. This was taken from Turtlesim code base where teleop takes [W A S D X] keys to control bot

    def __getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()
        
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key 
            



if __name__ == "__main__":

    """ 

    These are tests for the User Interface. 
    
    -> These commands are not to be executed while operating the robot. Instead this only serves as a place to
        write unit tests for certain functions

    """
    
    rospy.init_node('user_interface_test', anonymous=False)

    sectionSplitter = '----------' # UI Element [IGNORE]

    UserInterface()

 

