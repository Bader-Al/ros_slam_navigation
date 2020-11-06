import math

class CoordinatesUtils:


  def getCoordinates_sortedByDistance(unsortedGoalsList, currentBotLocation, parser):
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
        tempDict.update({'name' : goalName ,"coordinates": XY_coordinates, 'distance' : goalDistance} )
        dictList.append(tempDict)
    sortedListOfDictionaries = sorted(dictList, key = lambda i: i['distance'])
    return sortedListOfDictionaries


if __name__ == "__main__":
  print("Runing utils test")