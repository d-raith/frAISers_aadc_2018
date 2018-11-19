
'''
*******************************************************************************
  Audi Autonomous Driving Cup 2018
  Team frAIsers
*******************************************************************************
'''



""" helpers """
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    GREEN_BG = '\033[42m'
    OKYELLOW = '\033[33m'
    YELLOW_BG = '\033[43m'
    OKRED = '\033[31m'
    RED_BG = '\033[41m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    TEXT_BLACK = '\033[30m'
    INVERT = '\033[7m'


""" print bold to the console """
def printBold(string):
    return bcolors.BOLD + string + bcolors.ENDC


""" print green to the console """
def printGreen(string, bg=False):
    if not bg:
        return bcolors.OKGREEN + string + bcolors.ENDC
    else:
        return bcolors.TEXT_BLACK + bcolors.GREEN_BG + string + bcolors.ENDC


""" print yellow to the console """
def printYellow(string, bg=False):
    if not bg:
        return bcolors.OKYELLOW + string + bcolors.ENDC
    else:
        return bcolors.TEXT_BLACK + bcolors.YELLOW_BG + string + bcolors.ENDC


""" print red to the console """
def printRed(string, bg=False):
    if not bg:
        return bcolors.OKRED + string + bcolors.ENDC
    else:
        return '\033[7m' + bcolors.OKRED + string + bcolors.ENDC