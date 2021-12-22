#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#      __  ___          __    ____        ____  _____
#     /  |/  /__  _____/ /_  / __ \____ _/ __ \/ ___/
#    / /|_/ / _ \/ ___/ __ \/ /_/ / __ `/ / / /\__ \ 
#   / /  / /  __(__  ) / / / _, _/ /_/ / /_/ /___/ / 
#  /_/  /_/\___/____/_/ /_/_/ |_|\__,_/_____//____/  
#  
#
#  Copyright (C) 2021 MeshRaDs
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script allows the user to connect to several crazyflies at once
Afterwards a predetermined sequence can be flewn by the drones, similary to the synchronizedSequence.py from the example folder
During this maneuvers parameters from each drone cann be logged and also the debugging_prints out of the crazyflie hardware are displayed
"""
from math import pi
import threading
import time
from collections import namedtuple
from queue import Queue

# Drone Authentification and Swarm
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


# Possible commands, all times are in seconds
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Goto = namedtuple('Goto', ['x', 'y', 'z', 'time'])
Turn = namedtuple('Turn', ['degrees'])
Quit = namedtuple('Quit', [])                          # Reserved for the control loop, do not use in sequence
Measurment = namedtuple('Measurment', ['time'])                  # Waits until amount many P2P packets have been received
Wait_for_landing = namedtuple('Wait_for_landing', [])



# Configuration Variables
uris = [
    'radio://0/80/2M/E7E7E7E704',  # cf_id 0
    'radio://0/80/2M/E7E7E7E703',  # cf_id 1
    #'radio://0/80/2M/E7E7E7E702',  # cf_id 2
    # More URIs can be added to have more drones within the swarm
]

# Time to wait between two steps
STEP_TIME = 2


sequence = [
    # Step, CF_id,  action
    (0,    0,      Takeoff(0.5, 2)),

    (1,    1,      Takeoff(0.5, 2)),

    (2,    1,      Goto(0, 0, 0.5, 2)),

    (3,    0,      Goto(0, -1, 0.5, 2)),

    (4,    1,      Wait_for_landing()),
    (4,    0,      Measurment(2)),

    (5,    0,      Goto(0, -1, 0.75, 2)),

    (6,    0,      Measurment(2)),

    (7,    0,      Land(2)),
    (7,    1,      Land(2)),

]


# Logging
from cflib.crazyflie.log import LogConfig

def log_async(scf):
    log_config = LogConfig(name='Position', period_in_ms=100)   # In here any data can be set to be logged
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_config)
    log_config.data_received_cb.add_callback(lambda t, d, l: logging_callback(scf.cf.link_uri, t, d, l))
    log_config.start()


def logging_callback(uri, timestamp, data, log_conf):       # The callback gets executed whenever new logging data is received
    x = float(data['stateEstimate.x'])
    y = float(data['stateEstimate.y'])
    z = float(data['stateEstimate.z'])
    print("Postion:{}, {}, {}" .format(x, y, z) )

# Logging console text

logging_list = []
global fligth_status
fligth_status = False

def firmware_print_callback(console_text):              # this callbacks gets called whenever a print message gets received from a crazyflie
     #print(console_text, end = '')
    logging_list.append(console_text)
    
   
def logging_writer(file_name):
    print("Logging data into: {}".format(file_name))
    with open(file_name, 'w') as logging_file:
        already_writing = False
        for log in logging_list:

            if ("P2P" in log) or already_writing:
                already_writing = True
                for char in log:
                    if char == '\n':
                        logging_file.write(';')
                        already_writing = False
                    logging_file.write(char)



# Definitions of Functions used for the swarm to fly
def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', str(controller))


def crazyflie_control(scf):
    cf = scf.cf
    control = controlQueues[uris.index(cf.link_uri)]

    activate_mellinger_controller(scf, False)

    commander = scf.cf.high_level_commander
    global logging_count
    global fligth_status

    step = 0

    while True:
        command = control.get()
        print('\nStep {}:\n'.format(step))
        print(' - Running: {} on Drone {}'.format(command, cf.link_uri[-1]))
        if type(command) is Quit:
            print('\nReaching the end of the sequence, stopping!')
            return
        elif type(command) is Takeoff:
            fligth_status = True          
            commander.takeoff(command.height, command.time)
            time.sleep(2)
        elif type(command) is Land:
            fligth_status = False
            commander.land(0.0, command.time)
        elif type(command) is Goto:
            commander.go_to(command.x, command.y, command.z, 0,  command.time)
            logging_list.append("P2P:{} reached Position {}, {}, {}\n".format(cf.link_uri[-1], command.x, command.y, command.z))
        elif type(command) is Turn:
            commander.go_to(0, 0, 0, ((pi/180)*command.degrees), 1, relative=True)
            logging_list.append("P2P:{} turned by {}°\n".format(cf.link_uri[-1], command.degrees))
        elif type(command) is Measurment:
            time.sleep(3)
            logging_list.append("P2P:{} starting measurment\n" .format(cf.link_uri[-1]))
            time.sleep(command.time)
            logging_list.append("P2P:{} finished measurment\n" .format(cf.link_uri[-1]))
        elif type(command) is Wait_for_landing:
            while fligth_status:
                time.sleep(1)
        else:
            print('Warning! unknown command {} for uri {}'.format(command,cf.link_uri[-1]))
                                                                  
        step += 1
        time.sleep(STEP_TIME)


def control_thread():
    pointer = 0
    step = 0
    stop = False

    #clearing the queu 
    for id in range(len(uris)):
        while not controlQueues[id].empty():
                controlQueues[id].get(False)

        

    while not stop:
        while sequence[pointer][0] <= step:
            cf_id = sequence[pointer][1]
            command = sequence[pointer][2]
            controlQueues[cf_id].put(command)

            pointer += 1

            if pointer >= len(sequence):
                stop = True
                break

        step += 1
        time.sleep(1)

    for ctrl in controlQueues:
        ctrl.put(Quit())


if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        for uri in uris:
            swarm._cfs[uri].cf.console.receivedChar.add_callback(firmware_print_callback)       # adding callback for each cf
        
        # swarm.parallel_safe(log_async)                    # cb for logger
        swarm.parallel_safe(activate_high_level_commander)
        swarm.reset_estimators()

        print('Starting sequence and logging measurments!')
        logging_list = []      


        threading.Thread(target=control_thread).start()
        swarm.parallel_safe(crazyflie_control)


        time.sleep(1)
        logging_writer('logging.csv')


