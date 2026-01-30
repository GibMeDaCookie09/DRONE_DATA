from pymavlink import mavutil
import asyncio
import websockets
import json
import time

# =========================
# CONFIG
# =========================
ESP_IP = "0.0.0.0"
ESP_PORT = 14550

WS_HOST = "0.0.0.0"
WS_PORT = 8765

HEARTBEAT_TIMEOUT = 3.0  # seconds

# =========================
# MAVLINK CONNECT (UDP)
# =========================
print("📡 Waiting for MAVLink over WiFi...")

master = mavutil.mavlink_connection(
    f"udp:{ESP_IP}:{ESP_PORT}"
)

master.wait_heartbeat()
print("✅ MAVLink heartbeat received")

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

request(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30)
request(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
request(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
request(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)

# =========================
# STATE
# =========================
last_heartbeat = time.time()

telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "throttle": 0,
    "battery": 0.0,
    "lat": None,
    "lng": None,
    "heartbeat": False,
    "path": "WiFi"
}

clients = set()

# =========================
# MAVLINK LOOP
# =========================
async def mavlink_loop():
    global last_heartbeat

    while True:
        msg = master.recv_match(blocking=False)
        if msg:
            t = msg.get_type()

            if t == "HEARTBEAT":
                last_heartbeat = time.time()

            elif t == "ATTITUDE":
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

        # Heartbeat status update
        telemetry["heartbeat"] = (time.time() - last_heartbeat) < HEARTBEAT_TIMEOUT

        await asyncio.sleep(0)

# =========================
# TELEMETRY BROADCAST
# =========================
async def broadcast_loop():
    while True:
        if clients:
            payload = json.dumps(telemetry)
            await asyncio.gather(
                *[c.send(payload) for c in list(clients)],
                return_exceptions=True
            )
        await asyncio.sleep(1 / 60)

# =========================
# COMMAND HANDLER
# =========================
def handle_command(cmd):
    print(f"📥 Command: {cmd}")

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

# =========================
# WEBSOCKET HANDLER
# =========================
async def ws_handler(ws):
    print("🌐 Web client connected")
    clients.add(ws)
    try:
        async for msg in ws:
            data = json.loads(msg)
            if "command" in data:
                handle_command(data["command"])
    finally:
        clients.remove(ws)
        print("❌ Web client disconnected")

# =========================
# MAIN
# =========================
async def main():
    print(f"🚀 WebSocket running at ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.gather(
            mavlink_loop(),
            broadcast_loop()
        )

asyncio.run(main())
