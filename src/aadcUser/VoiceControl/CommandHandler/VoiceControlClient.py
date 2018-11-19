
'''
*******************************************************************************
  Audi Autonomous Driving Cup 2018
  Team frAIsers
*******************************************************************************
'''


""" IMPORTS """
import sys
import glob
sys.path.append('../VoiceControlThrift/gen-py')
from control_thrift import ControlComm as control_thrift
import numpy as np
from queue import PriorityQueue
import time

# thrift
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol

# AWS
import boto3

import SqsClient as sqs

from CommandHandler import *
from helpers import *


""" GLOBAL VARIABLES """
DEBUG = True
SQS_POLL_TIMEOUT = 2        # in seconds


# server ids
car26_id = control_thrift.car26_id
car27_id = control_thrift.car27_id
# server ips
car26_ip = "192.168.168.126"
car27_ip = "192.168.168.127"
control_server_port = "9095"

car_ips = { car26_id : car26_ip, car27_id : car27_ip}


#______________________________________________________________________________



""" FUNCTIONS """



""" main """
def main():
    print("\n\n{:^80}\n".format(printBold("***** AADC Voice Control Client *****")))
    print("\n{:^80}\n".format(printBold("((( o )))")))
    # print('\nConnect to:\t[a] ' + bcolors.UNDERLINE + 'Car 26'  + bcolors.ENDC + '\t[b] ' + bcolors.UNDERLINE + 'Car 27'  + bcolors.ENDC +'\t[l] ' + bcolors.UNDERLINE + 'localhost' + bcolors.ENDC +'\n')

    command_handler = CommandHandler(car_ips, control_server_port, SQS_POLL_TIMEOUT)
    
    try:
        while True:
            # STEP 1: connect to the car servers
            s1 = ">>> Stage 1/3: Connect to Cars <<<" + bcolors.ENDC
            print(bcolors.OKGREEN + "\n________________________________________________________________________________\n")
            print("\n{:^80}\n".format(s1))
            
            connected = command_handler.connectToCars()

            if not connected:
                print(bcolors.WARNING + "no connected server found!" + bcolors.ENDC)
                continue
            else:
                print("\nConnected.")
            
            command_handler.printConnectionStats()
            command_handler.printCarStats()

            # STEP 2: connect to AWS
            s2 = ">>> Stage 2/3: Connect to AWS <<<" + bcolors.ENDC
            print(bcolors.OKGREEN + "\n________________________________________________________________________________\n")
            print("\n{:^80}\n".format(s2))

            command_handler.connectToAWS()

            # STEP 3: start the VoiceControl service
            s2 = ">>> Stage 3/3: Connect to AWS <<<" + bcolors.ENDC
            print(bcolors.OKGREEN + "\n________________________________________________________________________________\n")
            print("\n{:^80}\n".format(s2))

            command_handler.startHandler()
                
    except KeyboardInterrupt:
        print('\nexiting.')
        exit()

if __name__ == '__main__':
    main()

""" parse the input parameters """
def parseArgs():
    if len(sys.argv) != 2:
        print('Usage: python3 ThriftClient.py <filename>')
        exit()
    else:
        return sys.argv[1]