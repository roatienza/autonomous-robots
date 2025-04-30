'''
Sample python REST API

Move to cartesian coordinates (0.5, 0, 0.3) in 3 seconds:
    python3 rest.py --parameters 0.5 0 0.3 3.0
    
Close the gripper:
    python3 rest.py --command closeGripper --parameters 0.0

Open the gripper:
    python3 rest.py --command openGripper --parameters 0.1

'''

import requests
import sys
import json
import argparse

def send_floats(ip, port, command, parameters):
    url = f"http://{ip}:{port}/api/floats"
    data = {
        command: parameters
    }

    response = requests.post(url, json=data)
    return response

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send three floating point numbers to the server.")
    parser.add_argument('--time', type=float, default=5., help="The time to execute the command.")
    parser.add_argument('--parameters', type=float, nargs='*', help="An array of floating point numbers representing coordinates.")
    parser.add_argument('--ip', type=str, default="192.168.2.1", help="The server's IP address")
    parser.add_argument('--port', type=int, default=34568, help="The server's port number")
    parser.add_argument('--command', type=str, default="moveToCartesian", help="The command to send to the server")
    args = parser.parse_args()

    #print(args.parameters)
    #exit(0)
    #x, y, z = args.parameters
    #t = args.time
    #assert x is not None and y is not None and z is not None, "Please provide three coordinates."
    #assert t is not None and t > 1.0, "Please provide a positive time."

    ip = args.ip
    port = args.port
    
    print(f"Sending coordinates: {args.parameters}")
    print(f"Server IP: {ip}")
    print(f"Server Port: {port}")

    response = send_floats(ip, port, args.command, args.parameters)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.json()}")
