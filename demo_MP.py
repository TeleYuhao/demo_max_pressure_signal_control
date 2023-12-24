# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 09:42:21 2022

@author: yhd
"""

import numpy as np
import pandas as pd
import traci
import sys, subprocess, os
import inspect
from read_net import NetworkData


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    import sumolib
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
net = NetworkData('intersection/environment.net.xml')

def phase_lane_get(net):
    phase_1,phase_2,phase_3,phase_4 = [],[],[],[]
    phase = []
    for i in net.lane_data:
        if net.lane_data[i]['edge'] == 'E2TL' or net.lane_data[i]['edge'] == 'W2TL':
            if net.lane_data[i]['movement'] == 's' or net.lane_data[i]['movement'] == 'rs':
                phase_1.append(i)
            else:
                phase_2.append(i)
        else:
            if net.lane_data[i]['movement'] == 's' or net.lane_data[i]['movement'] == 'rs':
                phase_3.append(i)
            elif net.lane_data[i]['movement'] == 'l':
                phase_4.append(i)
            else:
                continue
            
    phase.append(phase_1)
    phase.append(phase_2)
    phase.append(phase_3)
    phase.append(phase_4)
    
    return phase

def pressure_cal(phase,net):
    total_pressure = 0
    for income_lane in phase:
        income_pressure = traci.lane.getLastStepVehicleNumber(income_lane)
        outcome_pressure = traci.lane.getLastStepVehicleNumber(next(iter(net.lane_data[income_lane]['outgoing'])))
        
        total_pressure += income_pressure - outcome_pressure
        #print(total_pressure)
    return total_pressure/len(phase)

def max_pressure_phase(phase):
    pressure = []
    for j in phase:
        #print(j)
        pressure.append(pressure_cal(j,net))

    pressure = np.array(pressure)
    phase_index = pressure.argmax()
    if phase_index == 0:
        phase_index = 4
    elif phase_index == 1:
        phase_index = 6
    elif phase_index == 2:
        phase_index = 0
    else :
        phase_index = 2
    return phase_index

def max_pressure_controller(phase_pressure_max,phase_now,current_green_time,current_time):
    min_green = 16
    max_green = 45
    phase_extend_time = 5
    if phase_pressure_max == phase_now:
        phase_next = phase_now
    else:
        phase_next = phase_now + 2
    if phase_next > 7:
        phase_next -=8
    if phase_now == phase_next and current_green_time <max_green:
        traci.trafficlight.setPhase('TL',phase_now)
        traci.trafficlight.setPhaseDuration('TL',phase_extend_time)
        current_green_time = current_green_time + phase_extend_time
        next_swith_time = current_time + phase_extend_time -1
        
        print(current_time,'相位延长',next_swith_time,phase_now)
    elif (phase_now == phase_next and current_green_time >=max_green) or (phase_now != phase_next):
        if phase_now + 1 <=7 :
            yellow_phase = phase_now + 1
        else:
            yellow_phase = 0
        
        traci.trafficlight.setPhase('TL',yellow_phase)
        traci.trafficlight.setPhaseDuration('TL',3)
        for step in range(3):
            traci.simulationStep()
        traci.trafficlight.setPhase('TL',phase_next)
        traci.trafficlight.setPhaseDuration('TL',min_green) 
        current_green_time = min_green -1
        next_swith_time = current_time + min_green + 3
        print(current_time,'切换相位',next_swith_time,phase_now)
        
    print(phase_next)
        
    return current_green_time , next_swith_time


def main():
    current_green_time = 15
    next_swith_time = 200
    phase = phase_lane_get(net)
    traci.start(["sumo-gui", "-c", "intersection/sumo_config.sumocfg"])
    for step in range(3600):
        traci.simulationStep()
        current_time = traci.simulation.getTime()
        if current_time < 200 :
            continue
        elif current_time >3600:
            break
        else:
            if current_time == next_swith_time:
                phase_pressure_max = max_pressure_phase(phase)
                phase_now = traci.trafficlight.getPhase('TL')
                current_time = traci.simulation.getTime()
                current_green_time,next_swith_time = max_pressure_controller(phase_pressure_max,phase_now,current_green_time,current_time)
            else:
                continue
    traci.close()
            
if __name__=='__main__':
    main()
    
    