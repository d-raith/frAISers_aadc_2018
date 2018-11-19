
'''
*******************************************************************************
  Audi Autonomous Driving Cup 2018
  Team frAIsers
*******************************************************************************
'''


import sys
import glob
sys.path.append('../VoiceControlThrift/gen-py')

from queue import PriorityQueue
import time
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from control_thrift import ControlComm as control_thrift
import boto3

import SqsClient as sqs
from helpers import *



class CommandHandler():

    def __init__(self, car_ips, port, timeout):
        self.client_handle = None
        self.car_connected = True
        self.aws_connected = False
        self.connected_cars = {}
        self.aws_client = None

        self.port = port
        self.car_ips = car_ips
        self.car26_id = list(car_ips.keys())[0]
        self.car27_id = list(car_ips.keys())[1]
        self.timeout = timeout

        

    """ connect to the Car MapServer via Thrift """
    def connectToControlServer(self, host, port):
        try:
            print('\nControlClient: trying to connect to Car ThriftServer at {} on port {}'.format(host, port))
            transport = TSocket.TSocket(host, port)
            protocol = TBinaryProtocol.TBinaryProtocol(transport)
            client = control_thrift.Client(protocol)
            transport.open()
            print(printGreen('ControlClient: transport opened!'))
            return [client, transport]
        except:
            print("\n" + printRed('ControlClient: could not connect!', True) + "\n")
            return [False, False]


    """ create a new connection to a car control server """
    def createCarInstance(self, car_id):
        if car_id in self.car_ips.keys():
            [car_handle, car_transport] = self.connectToControlServer(self.car_ips[car_id], self.port)
            if car_handle:
                self.connected_cars[car_id] = {"client_handle":car_handle, "connected":True}
                return True
        return False


    """ check if car is still connected """
    def checkCarConnection(self, car_id, reconnect=False):
        if car_id in self.connected_cars.keys():
            if self.connected_cars[car_id]["connected"]:
                try:
                    self.connected_cars[car_id]["client_handle"].ping()
                    return True
                except:
                    print("ping for %s failed!", car_id)
                    self.connected_cars[car_id] = {"client_handle":car_handle, "connected":False}
            if reconnect:
                connected = self.connectToCar(car_id)
                if connected:
                    try:
                        self.connected_cars[car_id]["client_handle"].ping()
                        self.connected_cars[car_id] = {"client_handle":car_handle, "connected":True}
                        return True
                    except:
                        print("ping for %s failed!", car_id)
                        return False
                else:
                    print("reconnect for %s failed!", car_id)
                    return False
        else:
            print("car_id not in initialized cars")
        return


    """ get the status of the car from the car server """
    def getCarStatus(self, car_id):
        if self.connected_cars[car_id]["connected"]:
            try:
                self.connected_cars[car_id]["status"] = self.connected_cars[car_id]["client_handle"].getCarStatus()
            except TException:
                print("getCarStatus(): ThriftException occured")
                raise ValueError("TException")
            return True
        else:
            print("car %s not connected", car_id)
            return False


    """ connect to the car thrift servers """
    def connectToCars(self):
        s = "\n> Type:\t[ " + printBold("26") + " / " + printBold("27") + " / " + printBold("both") + \
            " ] to choose the ControlServers to connect to.\n\n\t[" + printBold("Strg+C") + \
            "] to exit\n\n\t[" + printBold("ENTER") + "] to confirm: "
        
        key = input(s)
        
        if (key == '26'):
            connected = self.createCarInstance(self.car26_id)
            if not connected:
                print(bcolors.WARNING + "Could not connect to Car 26!", bcolors.ENDC)
                return False
            return True
        elif (key == '27'):
            connected = self.createCarInstance(self.car27_id)
            if not connected:
                print(bcolors.WARNING + "Could not connect to Car 27!", bcolors.ENDC)
                return False
            return True
        elif (key == 'both'):
            connected = self.createCarInstance(self.car26_id)
            if not connected:
                print(bcolors.WARNING + "Could not connect to Car 26!", bcolors.ENDC)
                return False
            connected = self.createCarInstance(self.car27_id)
            if not connected:
                print(bcolors.WARNING + "Could not connect to Car 27!", bcolors.ENDC)
                return False
            return True
        else:
            print("\nInvalid input!")
            return False


    """ connect to AWS """
    def connectToAWS(self):
        s = "\n> Type:\t[ " + printBold("c") + " ] to connect to the AWS service\n\n\t[" + \
            printBold("Strg+C") + "] to abort\n\n\t[" + printBold("ENTER") + "] to confirm: "
        try:
            key = input(s)
            if (key == 'c'):
                if self.aws_client is None:
                    self.aws_client = sqs.SqsClient()
                    self.aws_connected = True
                    print(bcolors.OKGREEN + "AWS client created", bcolors.ENDC)
                else:
                    print("AWS client already exists")
                # client_handle.ping()
                try:
                    self.aws_client.init()
                except:
                    print("Could not initialize AWS client")
        except KeyboardInterrupt:
            print('\nshutting down VoiceControl service...\n\n')


    """ core run function """
    def run(self):
        print("CommandHandler: starting service")
        try:
            
            while True:
                cmd = self.aws_client.getNextCommand()
                if cmd:
                    print("CommandHandler: received command from AWS")
                    command = self.processAlexaCommand(cmd)
                    self.handleCommand(command)
                time.sleep(self.timeout)
        
        except KeyboardInterrupt:
            print("CommandHandler: returned from service")
            return
            



    """ launch VoiceControl handler """
    def startHandler(self):
        s = "\n> Type:\t[ " + printBold("s") + " ] to launch the VoiceControl service\n\n\t[" + \
            printBold("Strg+C") + "] to abort\n\n\t[" + printBold("ENTER") + "] to confirm: "
        try:
            key = input(s)
            if (key == 's'):
                print('launching...')
                if self.aws_client is None:
                    print("AWS client was not initialized")
                    return
                if not self.aws_client.is_initialized:
                    print("Initializing AWS client")
                    self.aws_client.init()
                else:
                    print("AWS client already initialized")
                
                self.run()
                
        except KeyboardInterrupt:
            print('\nshutting down VoiceControl service...\n\n')


    """ print the command details """
    def printCommandStats(self, command):
        indent = "\t"
        print(indent + "___Command details___")
        print(indent + "{:<10}{:}".format("OP", printBold(command["MessageOp"]._VALUES_TO_NAMES())))
        cars_ids = ""
        for car_id in command["car_ids"]:
            cars_ids += car_id + " "
        print(indent + "{:<10}{:}".format("Cars", printBold(cars_ids)))
        if command["override"]:
            print(indent + "{:<10}{:}".format("Override", printYellow("TRUE", True)))
        else:
            print(indent + "{:<10}{:}".format("Override", "false"))
        poi_xyz = "x: " + str(command["poi"].x) + " y: " + str(command["poi"].y) + " z: " + str(command["poi"].z)
        print(indent + "{:<10}{:}".format("POI", printBold(poi_xyz)))
        print("\n\n")
        
    
    """ print connection information """
    def printConnectionStats(self):
        indent = "\t"
        print(indent + "___Connection status summary___")
        for car_id in self.connected_cars.keys():
            if self.connected_cars[car_id]["connected"]:
                print(indent + "{:<15}{}".format(car_id, printGreen("Connected")))
            else:
                print(indent + "{:<15}{}".format(car_id, printRed("Not connected")))

        if self.aws_connected:
            print("{:<15}{}".format("AWS:", "Connected"))
        else:
            print("{:<15}{}".format("AWS:", "Not connected"))
        print("\n\n")


    """ print the current status of the cars """
    def printCarStats(self):
        indent = "\t"
        print(indent + "___Car status summary___")
        for car_id in self.connected_cars.keys():
            print(indent + "{:<15}{}".format(car_id, self.printCarStatus(car26_status)))
        print("\n\n")


    """ return the car status as string """
    def printCarStatus(self, car_status):
        if car_status is control_thrift.CarStatus.READY:
            return printGreen("READY")
        elif car_status is control_thrift.CarStatus.ON_ROUTE:
            return printGreen("ON ROUTE", True)
        if car_status is control_thrift.CarStatus.WAITING_FOR_INTERACTION:
            return printYellow("WAITING FOR INTERACTION", True)
        if car_status is control_thrift.CarStatus.ERROR:
            return printRed("ERROR", True)
        if car_status is control_thrift.CarStatus.UNINITIALIZED:
            return printRed("UNINITIALIZED")


    """ check the status of each initialized car """
    def checkAllCarStatus(self):
        for car_id in self.connected_cars.keys():
            self.checkCarStatus(car_id, False)


    """ check if still connected to the car server """
    def checkCarStatus(self, car_id, reconnect=False):
        try:
            car26_client.ping()
        except:
            car26_connected = False
            print(bcolors.WARNING + "Car 26 server gone." + bcolors.ENDC)
        try:
            car27_client.ping()
        except:
            car27_connected = False
            print(bcolors.WARNING + "Car 27 server gone." + bcolors.ENDC)
        
        if reconnect:
            print("Reconnecting...")
            [car26_client, car26_transport] = self.connectToControlServer(car26_ip, control_server_port)
            [car27_client, car27_transport] = self.connectToControlServer(car27_ip, control_server_port)
            self.checkCarStatus(False)


    """ process the incoming command from AWS """
    def processAlexaCommand(self, cmd):
        command = {}
        try:
            # control op
            if cmd["op"] is "DRIVE":
                command.insert({"MessageOp":control_thrift.MessageOp.DRIVE})
            elif cmd["op"] is "PAUSE":
                command.insert({"MessageOp":control_thrift.MessageOp.PAUSE})
            elif cmd["op"] is "STOP":
                command.insert({"MessageOp":control_thrift.MessageOp.STOP})
            elif cmd["op"] is "RETURN_TO_START":
                command.insert({"MessageOp":control_thrift.MessageOp.RETURN_TO_START})
            elif cmd["op"] is "ABORT":
                command.insert({"MessageOp":control_thrift.MessageOp.ABORT})
            elif cmd["op"] is "PLAN":
                command.insert({"MessageOp":control_thrift.MessageOp.PLAN})
            elif cmd["op"] is "COMPUTE_COSTS":
                command.insert({"MessageOp":control_thrift.MessageOp.COMPUTE_COSTS})
            
            # command recepient
            car_ids = []
            for car_id in cmd["cars"]:
                if car_id is "26":
                    car_ids.append(control_thrift.car26_id)
                if car_id is "27":
                    car_ids.append(control_thrift.car27_id)
            command.insert({"car_ids":car_ids})

            # override exising command
            override = False
            if cmd["override"]:
                override = True
            command.insert({"override":override})

            # poi
            poi = control_thrift.Point3D()
            poi.x = float(cmd["poi"]["x"])
            poi.y = float(cmd["poi"]["y"])
            poi.z = float(cmd["poi"]["z"])
            command.insert({"poi":poi})

            return command
                
        except:
            print("corrupted aws command")
        return False
    

    """ handle internal commands """
    def handleCommand(self, command):
        # DRIVE
        if command["op"] is control_thrift.MessageOp.DRIVE:
            self.driveToPoi(command)
        
        # PAUSE
        if command["op"] is control_thrift.MessageOp.PAUSE:
            self.pauseCar(command)
        
        # STOP
        if command["op"] is control_thrift.MessageOp.PAUSE:
            self.stopCar(command)
        
        
        # TODO implement all cases


    """ prepare a control_thrift message to be send """
    def createThriftControlMessage(self, car_id, cmd, extra=""):
        control_msg = control_thrift.ControlMessage(cmd["MessageOp"], car_id, cmd["poi"], extra)
        return control_msg


    """ get the closest car for a specific POI """
    def getClosestCars(self, poi):
        cost_car_lookup = {}
        costs_car = PriorityQueue()
        for car_id in self.connected_cars.keys():
            costs = self.connected_cars[car_id]["client_handle"].getRouteCosts(car_id, control_thrift.MessageOp.COMPUTE_COSTS, poi)
            if costs >= 0:
                costs_car.put((costs, car_id))
        
        if not costs_car.empty():
            return costs_car
        return False


    """ send the thrift command to the car server """
    def sendCommandToCar(self, car_id, cmd, reconnect=False):
        thrift_cmd = createThriftControlMessage(car_id, cmd)
        print("> sending command")
        printCommandStats(cmd)

        if car_id in self.connected_cars.keys():
            try:
                if self.connected_cars[car_id]["connected"]:
                    return processCarResponse(self.connected_cars[car_id]["client_handle"].sendCommand(thrift_cmd), cmd, car_id)
                else:
                    if reconnect:
                        print("sendCommandToCar(): trying to reconnect...")
                        connected = self.checkCarConnection(car_id)
                        if connected:
                            return processCarResponse(self.connected_cars[car_id]["client_handle"].sendCommand(thrift_cmd), cmd, car_id)
                        else:
                            print("sendCommandToCar(): could not reconnect")
                            return False
                    else:
                        print("sendCommandToCar(): car not connected and reconnect disabled")
                        return False
            except TException:
                print(printRed("Thrift exception occured", True))
                raise ValueError("TException")
                
                
        else:
            print("car_id not initialized cars")
            return False

    
    """ check if a car is ready to drive """
    def checkCarReady(self, car_id, update=True):
        if update:
            connected = self.getCarStatus(car_id)
        return self.connected_cars[car_id]["status"] is control_thrift.CarStatus.READY
    
    
    """ let the car drive to a poi """
    def driveToPoi(self, command):
        if len(command["car_ids"]) > 0:
            # one car
            if len(command["car_ids"]) == 1:
                for car_id in command["car_ids"]:
                        possible = self.sendCommandToCar(car_id, command)
            
            # multiple cars
            else:
                # assign the closest car
                if not command["override"]:
                    closest_cars = self.getClosestCars(command["poi"])
                    while not closest_cars.empty():
                        closest_car_id = closest_cars.pop()
                        if checkCarReady(closest_car_id):
                            print("car %s is ready", closest_car_id)
                            possible = self.sendCommandToCar(closest_car_id, command)
                            if possible:
                                break
                        else:
                            print("car %s is already on route and override is false. proceeding with next closest car", closest_car_id)
                            continue
                    if not possible:
                        print("no car found for poi")
                
                # assign all cars
                else:
                    for car_id in command["car_ids"]:
                        possible = self.sendCommandToCar(car_id, command)
                        if not possible:
                            print("driveToPoi() not possible for %s", car_id)
                
            if possible:
                print("driveToPoi() succeeded")
                return True
            else:
                print("driveToPoi() failed")
                return False
        else:
            print("no car_id provided")
            return True


    """ pause cars """
    def pauseCars(self, command):
        for car_id in command["car_ids"]:
            paused = self.sendCommandToCar(car_id, command)
    

    """ resume cars """
    def resumeCars(self, command):
        for car_id in command["car_ids"]:
            resumed = self.sendCommandToCar(car_id, command)


    """ send STOP message to car """
    def stopCars(self, car_id):
        for car_id in command["car_ids"]:
            stopped = self.sendCommandToCar(car_id, command, True)


    """ process the return type of the thrift command """
    def processCarResponse(self, response, command, car_id):
        self.connected_cars[car_id]["status"] = response
        if command["MessageOp"] is control_thrift.MessageOp.DRIVE:
            if response is control_thrift.CarStatus.ON_ROUTE:
                print("car %s is ON_ROUTE", car_id)
                return True
            elif response is control_thrift.CarStatus.NO_ROUTE_FOUND:
                print("car %s has not found a route to the POI", car_id)
                return False
            else:
                return False
        if command["MessageOp"] is control_thrift.MessageOp.PAUSE:
            if response is control_thrift.CarStatus.PAUSED:
                print("car %s is PAUSED", car_id)
                return True
            else:
                print("car %s status is %s", car_id, response._VALUES_TO_NAMES())
                return False
        if command["MessageOp"] is control_thrift.MessageOp.CONTINUE:
            if response is control_thrift.CarStatus.ON_ROUTE:
                print("car %s is ON_ROUTE", car_id)
                return True
            else:
                print("car %s status is %s", car_id, response._VALUES_TO_NAMES())
                return False
        if command["MessageOp"] is control_thrift.MessageOp.STOP:
            if response is control_thrift.CarStatus.READY:
                print("car %s is stopped and ready for new commands", car_id)
                return True
            else:
                print("car %s status is %s", car_id, response._VALUES_TO_NAMES())
                return False
        
        return True
