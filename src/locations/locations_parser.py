#!/usr/bin/env python  
import rospy
import time
import yaml
from pathlib import Path

from geometry_msgs.msg import Point 



class LocationsParser:
    def __init__(self, file_path = "sim/house_locations.yaml", ):
        self.relative_path = file_path
        my_path = Path(__file__).resolve()  # resolve to get rid of any symlinks
        self.locations_path = my_path.parent / file_path
        self.locationsDictionary = self.__loadDictionaryFromYaml()
 
    def getRelativePath(self):
        return self.relative_path

    def find_Location(self, desiredLocation_name):
        assert desiredLocation_name.strip() != "" , "Location name is empty!"
        possibleLocationNames = set()
        for name in self.locationsDictionary['positions']:
            if name == desiredLocation_name.strip(): return name
            elif name in desiredLocation_name.strip() or desiredLocation_name.strip() in name:
                possibleLocationNames.add(name)
        return possibleLocationNames

    def insert_Location(self,location_name, coordinates_point ):
        assert  location_name.strip() != "" , "Location name is empty!"
        # assert  X, Y 
        coordinatesDict = {'x':coordinates_point.x,'y':coordinates_point.y, 'z':coordinates_point.z}
        self.locationsDictionary['positions'].update({location_name : coordinatesDict })
        with open( self.locations_path  , 'w') as outfile:
          print('writing to file')
          yaml.dump(self.locationsDictionary, outfile, default_flow_style=False)

    def delete_Location(self, location_name):
        assert location_name.strip() != "" , "Location name is empty!" 
        del self.locationsDictionary['positions'][location_name]
        with open( self.locations_path  , 'w') as outfile:
         print ('writing to file')
         yaml.dump(self.locationsDictionary, outfile, default_flow_style=False) 

    def get_XY_Coordinates(self, location_name ):
        assert location_name.strip() != "" , "Location name is empty!"
        return self.__getCoordinates(self.locationsDictionary['positions'],location=location_name )


    def get_positionNames(self):
        positionNames = set()
        for positionName in self.locationsDictionary['positions']:
            positionNames.add(positionName)
        return positionNames


    def dump_positions(self):
        for positionName in self.locationsDictionary['positions']:
            # print(positionName)
            coordinatesList = self.locationsDictionary['positions'][positionName]
            print(positionName, 'X : ', coordinatesList['x'], '   Y : ', coordinatesList['y'])
            
     
    def __loadDictionaryFromYaml(self):
        with self.locations_path.open() as locations_file:
            locationsDictionary = yaml.safe_load(locations_file)
        return locationsDictionary

    def __getCoordinates(self, locations_dictionary, location ):
        locationDict = locations_dictionary[location]
        return [locationDict['x'] , locationDict['y']] 




if __name__ == "__main__":

    """ 

    These are tests for the location parser. 
    
    -> These commands are not to be executed while operating the robot. Instead this only serves as a place to
        write unit tests for certain functions

    """

    sectionSplitter = '----------' # UI Element [IGNORE]

    file_path = "sim/house_locations.yaml"

    parser  = LocationsParser(file_path=file_path)
    
    print("\n\nRunning parser test... If you're not expecting to see this please run a node for example [ goals_publisher | user_interface ] \n\n")


    print(parser.getRelativePath())


    #TESTING BASE FUNCTIONALITY [Retreiving Information]
    print('The following are all positions for the given path:', file_path)
    print(sectionSplitter)
    parser.dump_positions()
    print(sectionSplitter)



    #TESTING ADDING NEW ITEM
    appendingLocationName = 'fakeRoomz'
    print('\n\nNext test is to add this location to the DB:',appendingLocationName)  
    print( sectionSplitter )     
    try:
        parser.insert_Location(location_name=appendingLocationName,coordinates_point= Point(x=1, y=1, z=0))
        print("Inserted item into database with no issues faced...")
        parser.insert_Location(location_name=appendingLocationName,coordinates_point= Point(x=23, y=43, z=0))
        print("Overwritten from X = 4 , Y = 4        to      X = 23  ,  Y =  43")
        print(sectionSplitter)
        print(sectionSplitter)
    except:
        print("Something went  wrong!")
    parser.dump_positions()
    print(sectionSplitter)


    #TESTING DELETING THAT ADDED ITEM
    print('\n\nNext test is to delete room from the DB:',appendingLocationName)  
    print( sectionSplitter ) 
    try:parser.delete_Location(appendingLocationName)
    except:print('Something went wrong!')
    parser.dump_positions()
    print(sectionSplitter)



#   TESTING DELETING OF A NON-EXISTING ITEM
    nanItem = "HelloWorldLocation"
    print('\n\nNext test is to delete a non existing room from the DB:',nanItem)  
    print( sectionSplitter ) 
    try:parser.delete_Location(nanItem)
    except:print('Something went wrong!')
    parser.dump_positions()
    print(sectionSplitter)



    #TESTING QUERY SUGGESTIONS
    testLocationName = 'room'
    print('\n\nNext test is to find all similar locations for givin desired location:',testLocationName)
    suggestedLocationNames = parser.find_Location(testLocationName)
    if type(suggestedLocationNames) == set :  
     print('Listed below are the possible location names for "%s"' %testLocationName)
     print('\n%s' %sectionSplitter )
     for name in suggestedLocationNames:
        print(name)
     print(sectionSplitter,'\n')

    elif type(suggestedLocationNames) == str:
        print("Unique item found!", testLocationName )



    #TESTING QUERY FOR EXISTING ITEM
    testLocationName = 'study_room'
    print('\n\nNext test is to find all similar locations for givin desired location:',testLocationName)
    suggestedLocationNames = parser.find_Location(testLocationName)
    if type(suggestedLocationNames) == set() :  
     print('Listed below are the possible location names for "%s"' %testLocationName)
     print( sectionSplitter )
     for name in suggestedLocationNames:
        print(name)
        print(sectionSplitter)
    elif type(suggestedLocationNames) == str:
        print("Unique item found!", testLocationName )
    
    
    print('\nTests complete...\n')

