### This is not currently used in our project. It serves no purpose currently. We partially built this part of the interface to give the user the ability to select
### environments which is out of the scope of the project, therefore it has been discarded to this file 


selected_environment = "sim"
selected_location = "house"


def _setEnvironment(user_input):
    global selected_environment
    user_input = user_input.strip()
    if 'sim' in user_input or user_input == 1: 
        selected_environment = "sim"
        return True
    elif 'univ' in user_input or user_input == 2:
        selected_environment = 'university'
        return True
    else:
        print("\nInvalid Input!\n")
        return False


def _selectEnvironment():
    while True:
        print("Selected environment : ", selected_environment.capitalize())
        env_input = input("Would you like to proceed? \n\n [Y] - continue\n [N] - change environment\n\n")
        if 'y' in env_input.lower(): break
        if 'n' in env_input.lower(): 
            env_selection = input('\n\nPlease select from the available environments \n 1 - Sim\n 2 - University\n\n')
            if(_setEnvironment(env_selection)):break

def _selectLocation():
    print('\nPossible locations in this environment include')
    from os import listdir
    from os.path import isfile, join
    my_path = Path(__file__).resolve()  # resolve to get rid of any symlinks
    locations_path = my_path.parent / 'locations'/ selected_environment
    onlyfiles = [f for f in listdir(locations_path) if isfile(join(locations_path, f))]
    print(onlyfiles)
