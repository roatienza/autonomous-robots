
from fastmcp import Client

async def main():
    async with Client("http://192.168.1.129:8085/franka") as client:
        response = await client.call_tool('send_command_tool', {'command': 'moveToCartesian', 'parameters': [0.5, 0, 0.3, 5.0]})
        print("Response:", response)
    
if __name__ == "__main__":
    import asyncio
    asyncio.run(main())