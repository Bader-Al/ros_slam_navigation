import math

class CoordinatesUtils:


  def getCoordinates_sortedByDistance(unsortedGoalsList, currentBotLocation, parser):
      ### NOT FOR PRODUCTION USE !
      # delete later
      ''' 

    ___FLAWS OF IN THE USAGE OF THIS FUNCTION________________________________________________________________________________
    
    Synchronous Path Planning: The issue here is that that this function is called once and the optimal point may change 
                                during movement of bot either due to shifting geo-conditions or unexepected obstacles. 
                                Therefore, further iteration of this function should make it run in an asynchronous fashion   
    _________________________________________________________________________________________________________________________

      '''
      sortedList = CoordinatesUtils.__sortByDistance_returnListOfDicts(unsortedGoalsList, currentBotLocation)
      sortedCoordinates = []
      ### Sorted list is Ascending... meaning that the closest coordinate will appear first
      for goal in sortedList:
          tempDict = {}
          goalName = goal['name'] 
          XY_Coordinates = parser.get_XY_Coordinates(goalName)
          distance = goal['distance']
          tempDict.update({'name' : goalName, 'coordinates' : XY_Coordinates, 'distance':distance})
          sortedCoordinates.append(tempDict) 
      return sorted(sortedCoordinates, key = lambda i: i['distance'])


  def __sortByDistance_returnListOfDicts(unsortedGoalsList, currentBotLocation):
    ### NOT FOR PRODUCTION USE !
    ''' 

    ___FLAWS OF THIS TECHNIQUE_______________________________________________________________________________________________________

    Inadequate Sorting Mechanism: The sorting algorithm relies on a bird's eye view function of distance which may be good for ariel 
                                  applications, however in this case it is not optimal... 
                                  [1] - Closer points may require longer paths due to walls or large obstacles  

    _________________________________________________________________________________________________________________________________

    '''
    dictList = []
    for listItem in unsortedGoalsList:
        tempDict = {}
        goalName = listItem[0]
        XY_coordinates = listItem[1]
        goalDistance = abs(0.5 * math.sqrt(((XY_coordinates[0]-currentBotLocation.x ) ** 2) + ((XY_coordinates[1]-currentBotLocation.y) ** 2)))
        tempDict.update({'name' : goalName , 'distance' : goalDistance} )
        dictList.append(tempDict)
    sortedListOfDictionaries = sorted(dictList, key = lambda i: i['distance'])
    return sortedListOfDictionaries


if __name__ == "__main__":
  print("Runing utils test")