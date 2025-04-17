#!/usr/bin/env python3

# A python code to control the Eva arm with the EvaSDK - The arm goes to a certain pose and stops

from evasdk import Eva
host_ip = "192.168.187.1"
token = "2e49551cc7cb55556ee0468e30296755e9f2cf01"
eva = Eva(host_ip, token) # creating an instance of the Eva class to control the robotic arm

with eva.lock():
    eva.control_wait_for_ready()
    print("Executing motion!!")
    eva.control_go_to([-0.2617994, 0.7504916, -2.40855437, 0.19198622, -0.1396263, -1.88495559])  

