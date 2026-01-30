from pymavlink import mavutil
import asyncio
import websockets
import json
import time

# =========================
# CONFIG
# =========================
MAVLINK_CONN = 'COM8'      # USB Pixhawk
BAUDRATE = 115200

WS_HOST = "0.0.0.0"
WS_PORT = 8765

# =========================
# CONNECT MAVLINK
# =========================
print("🔌 Connecting to Pixhawk...")
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUDRATE)
master.wait_heartbeat()
print("✅ Heartbeat received")

# =========================
# REQUEST STREAMS
# =========================
def request(stream, rate):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        stream,
        rate,
        1
    )

request(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30)     # Attitude
request(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
request(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
request(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)

print("📡 MAVLink streams requested")

# =========================
# TELEMETRY CACHE
# =========================
telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "throttle": 0,
    "battery": 0.0,
    "lat": None,
    "lng": None,
    "path": "USB"
}

clients = set()

# =========================
# MAVLINK READER LOOP
# =========================
async def mavlink_loop():
    while True:
        msg = master.recv_match(blocking=False)
        if msg:
            t = msg.get_type()

            if t == "ATTITUDE":
                telemetry["roll"] = round(msg.roll * 57.2958, 2)
                telemetry["pitch"] = round(msg.pitch * 57.2958, 2)
                telemetry["yaw"] = round(msg.yaw * 57.2958, 2)

            elif t == "RC_CHANNELS":
                telemetry["throttle"] = msg.chan3_raw

            elif t == "SYS_STATUS":
                if msg.voltage_battery > 0:
                    telemetry["battery"] = round(msg.voltage_battery / 1000, 2)

            elif t == "GLOBAL_POSITION_INT":
                telemetry["lat"] = msg.lat / 1e7
                telemetry["lng"] = msg.lon / 1e7

        await asyncio.sleep(0)  # yield only

# =========================
# TELEMETRY BROADCAST LOOP
# =========================
async def broadcast_loop():
    while True:
        if clients:
            payload = json.dumps(telemetry)
            await asyncio.gather(
                *[c.send(payload) for c in list(clients)],
                return_exceptions=True
            )
        await asyncio.sleep(1 / 60)  # ~60 FPS

# =========================
# COMMAND HANDLER
# =========================
def handle_command(cmd):
    print(f"📥 Command received: {cmd}")

    if cmd == "ARM":
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    elif cmd == "DISARM":
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    elif cmd == "EMERGENCY_STOP":
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    elif cmd.startswith("MODE:"):
        mode = cmd.split(":")[1]
        if mode in master.mode_mapping():
            mode_id = master.mode_mapping()[mode]
            master.set_mode(mode_id)
            print(f"✈ Mode changed to {mode}")

# =========================
# WEBSOCKET HANDLER
# =========================
async def ws_handler(websocket):
    print("🌐 Web client connected")
    clients.add(websocket)

    try:
        async for msg in websocket:
            data = json.loads(msg)
            if "command" in data:
                handle_command(data["command"])

    finally:
        clients.remove(websocket)
        print("❌ Web client disconnected")

# =========================
# MAIN
# =========================
async def main():
    print(f"🚀 WebSocket running on ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.gather(
            mavlink_loop(),
            broadcast_loop()
        )

asyncio.run(main())
