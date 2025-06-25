'''

Framka MCP to REST API bridge.

Move to cartesian coordinates (0.5, 0, 0.3) in 5 seconds:
    python3 rest.py --parameters 0.5 0 0.3 5.0
    
Move to cartesian coordinates (0.6, 0, 0.2) in 5 seconds with gripper delta rotation angles 0 10 0 degrees:
    python3 rest.py --parameters 0.6 0 0.2 5.0 0 10 0
    
Close the gripper:
    python3 rest.py --command closeGripper --parameters 0.0

Open the gripper:
    python3 rest.py --command openGripper --parameters 0.1
    


'''

from fastmcp import FastMCP
import requests
import argparse

mcp = FastMCP("Franka MCP Server")
ip = "192.168.2.1"
port = 34568

@mcp.tool()
def send_command_tool(command: str, parameters: list) -> requests.Response:
    """
    Send a command with floating point parameters to the server.
    
    :param command: The command to send to the server.
    :param parameters: A list of floating point numbers representing coordinates or other parameters.
    :return: The response from the server.
    """
    global ip, port
    url = f"http://{ip}:{port}/api/floats"
    data = {
        command: parameters
    }

    response = requests.post(url, json=data)
    return response


if __name__ == "__main__":
    import argparse
    # create argument parser
    parser = argparse.ArgumentParser(description="Franka MCP Server")
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host address for the MCP server')
    parser.add_argument('--port', type=int, default=8085, help='Port for the MCP server')
    parser.add_argument('--path', type=str, default='/franka', help='Path for the MCP server')
    parser.add_argument('--transport', type=str, default='streamable-http', help='Transport method for the MCP server')
    args = parser.parse_args()
        
        
    mcp.run(transport=args.transport, host=args.host, port=args.port, path=args.path)
    