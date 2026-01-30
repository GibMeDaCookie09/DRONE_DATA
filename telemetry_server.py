from pymavlink import mavutil
import asyncio
import websockets
import json
import time

MAVLINK_CONN = 'udp:0.0.0.0:14550'
WS_PORT = 8765

master = mavutil.mavlink_connection(MAVLINK_CONN)
master.wait_heartbeat()
print("MAVLink connected")

clients = set()

async def telemetry_loop():
    while True:
        msg = master.recv_match(blocking=False)

        data = {
            "roll": 0,
            "pitch": 0,
            "yaw": 0,
            "throttle": 0,
            "battery": 0,
            "path": "WiFi"
        }

        if msg:
            if msg.get_type() == "ATTITUDE":
                data["roll"] = round(msg.roll * 57.3, 2)
                data["pitch"] = round(msg.pitch * 57.3, 2)
                data["yaw"] = round(msg.yaw * 57.3, 2)

            if msg.get_type() == "SYS_STATUS":
                data["battery"] = round(msg.voltage_battery / 1000.0, 2)

            if msg.get_type() == "RC_CHANNELS":
                data["throttle"] = msg.chan3_raw

        if clients:
            payload = json.dumps(data)
            await asyncio.gather(*[c.send(payload) for c in clients])

        await asyncio.sleep(0.05)

async def handler(websocket):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def main():
    async with websockets.serve(handler, "0.0.0.0", WS_PORT):
        await telemetry_loop()

asyncio.run(main())
