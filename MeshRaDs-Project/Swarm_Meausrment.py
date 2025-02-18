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
import threading
import time
from collections import namedtuple
from queue import Queue

# Drone Authentification and Swarm
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.swarm import firmware_print_writing

# Possible commands, all times are in seconds
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Goto = namedtuple('Goto', ['x', 'y', 'z', 'time'])
Quit = namedtuple('Quit', [])                          # Reserved for the control loop, do not use in sequence

# Configuration Variables
uris = [
    'radio://0/80/2M/E7E7E7E703',  # cf_id 0
    #'radio://0/80/2M/E7E7E7E702',  # cf_id 1
    #'radio://0/80/2M/E7E7E7E703',  # cf_id 2
    # More URIs can be added to have more drones within the swarm
]

STEP_TIME = 5           # Time waited after each step in seconds

sequence = [
    # Step, CF_id,  action
    (0,    0,      Takeoff(0.5, 2)),

    (1,    0,      Goto(0,  0,   0.5, 1)),

    (2,    0,      Goto(0.5, -1, 0.5, 2)),


    (3,    0,      Land(2)),
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

    while True:
        command = control.get()
        if type(command) is Quit:
            return
        elif type(command) is Takeoff:
            commander.takeoff(command.height, command.time)
        elif type(command) is Land:
            commander.land(0.0, command.time)
        elif type(command) is Goto:
            commander.go_to(command.x, command.y, command.z, 0, command.time)
        else:
            print('Warning! unknown command {} for uri {}'.format(command,
                                                                  cf.uri))


def control_thread():
    pointer = 0
    step = 0
    stop = False

    while not stop:
        print('Step {}:'.format(step))
        while sequence[pointer][0] <= step:
            cf_id = sequence[pointer][1]
            command = sequence[pointer][2]

            print(' - Running: {} on ID{}, which is Drone:{}'.format(command, cf_id, uris[cf_id][-2:]))
            controlQueues[cf_id].put(command)
            pointer += 1

            if pointer >= len(sequence):
                print('Reaching the end of the sequence, stopping!')
                stop = True
                break

        step += 1
        time.sleep(STEP_TIME)

    for ctrl in controlQueues:
        ctrl.put(Quit())


if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(log_async)
        swarm.parallel_safe(activate_high_level_commander)
        swarm.reset_estimators()

        print('Starting sequence!')

        threading.Thread(target=control_thread).start()

        swarm.parallel_safe(crazyflie_control)

        time.sleep(1)

        firmware_print_writing('logging.csv')



