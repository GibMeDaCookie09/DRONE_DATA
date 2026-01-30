from pymavlink import mavutil
import asyncio
import websockets
import json
import time

# ================= CONFIG =================
MAVLINK_CONN = "COM6"        # CHANGE to your Pixhawk COM
BAUDRATE = 115200

WS_HOST = "0.0.0.0"
WS_PORT = 8765

HEARTBEAT_TIMEOUT = 3.0
RECONNECT_DELAY = 2.0
BROADCAST_HZ = 10           # UI update rate

# ================= STATE =================
master = None
last_packet_time = None

telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "throttle": 0,
    "battery": 0.0,
    "lat": None,
    "lng": None,
    "heartbeat": False,
    "path": "USB",
    "reconnected": False,
    "tick": 0               # 🔥 keeps UI/logs alive
}

clients = set()

# ================= RESET TELEMETRY =================
def reset_telemetry():
    telemetry.update({
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "throttle": 0,
        "battery": 0.0,
        "lat": None,
        "lng": None,
        "heartbeat": False,
        "reconnected": True
    })

# ================= CONNECT MAVLINK =================
def connect_mavlink():
    global master, last_packet_time

    try:
        print("🔄 Connecting to Pixhawk...")
        master = mavutil.mavlink_connection(
            MAVLINK_CONN,
            baud=BAUDRATE,
            autoreconnect=False
        )

        master.wait_heartbeat(timeout=5)
        last_packet_time = time.time()

        reset_telemetry()

        print("✅ MAVLink connected")

        # Request streams
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

    except Exception as e:
        print("❌ MAVLink connect failed:", e)
        master = None
        last_packet_time = None

# ================= MAVLINK LOOP =================
async def mavlink_loop():
    global master, last_packet_time

    while True:
        if master is None:
            connect_mavlink()
            await asyncio.sleep(RECONNECT_DELAY)
            continue

        msg = master.recv_match(blocking=False)

        if msg:
            last_packet_time = time.time()
            telemetry["reconnected"] = False

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

        now = time.time()

        telemetry["heartbeat"] = (
            last_packet_time is not None and
            (now - last_packet_time) < HEARTBEAT_TIMEOUT
        )

        # HARD DISCONNECT
        if last_packet_time and (now - last_packet_time) > HEARTBEAT_TIMEOUT * 2:
            print("⚠️ MAVLink lost — reconnecting")
            try:
                master.close()
            except:
                pass
            master = None
            last_packet_time = None

        await asyncio.sleep(0.002)

# ================= BROADCAST LOOP =================
async def broadcast_loop():
    while True:
        telemetry["tick"] += 1     # 🔥 forces frontend updates
        if clients:
            payload = json.dumps(telemetry)
            for c in list(clients):
                await c.send(payload)
        await asyncio.sleep(1 / BROADCAST_HZ)

# ================= COMMANDS =================
def handle_command(cmd):
    if master is None:
        return

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

# ================= WEBSOCKET =================
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

# ================= MAIN =================
async def main():
    print(f"🚀 WebSocket running at ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.gather(
            mavlink_loop(),
            broadcast_loop()
        )

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("🛑 Server stopped cleanly")
