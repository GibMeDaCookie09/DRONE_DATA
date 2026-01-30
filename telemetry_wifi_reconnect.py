from pymavlink import mavutil
import asyncio
import websockets
import json
import time
import socket

# ================= CONFIG =================
MAVLINK_UDP = "udp:0.0.0.0:14550"

WS_HOST = "0.0.0.0"
WS_PORT = 8765

HEARTBEAT_TIMEOUT = 3.0
RECONNECT_DELAY = 1.0
BROADCAST_HZ = 30

# ================= STATE =================
master = None
last_packet_time = None
connected = False

clients = set()

telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "throttle": 0,
    "battery": 0.0,
    "lat": None,
    "lng": None,
    "heartbeat": False,
    "path": "WiFi",

    # CDAC PoC fields
    "event_context": "KUMBH_MELA_2025",
    "event_mode": "NORMAL",
    "last_seen_age_sec": None,
    "search_radius_m": None,
    "child_confidence": 0.0,
    "privacy_mode": True,
    "operator_note": "",

    "tick": 0
}

# ================= HARD RESET =================
def reset_connection():
    global master, last_packet_time, connected

    if master:
        try:
            master.close()
        except:
            pass

    master = None
    last_packet_time = None
    connected = False

    telemetry["heartbeat"] = False
    telemetry["event_mode"] = "NORMAL"

# ================= CONNECT MAVLINK =================
def connect_mavlink():
    global master, last_packet_time, connected

    try:
        print("🔄 Binding MAVLink UDP socket...")
        master = mavutil.mavlink_connection(
            MAVLINK_UDP,
            source_system=255,
            source_component=0
        )

        master.wait_heartbeat(timeout=5)
        last_packet_time = time.time()
        connected = True

        print("✅ MAVLink connected")

        # ALWAYS re-request streams on reconnect
        def request(stream, rate):
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream,
                rate,
                1
            )

        request(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 40)
        request(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 15)
        request(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5)
        request(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)

    except Exception as e:
        print("❌ MAVLink bind failed:", e)
        reset_connection()

# ================= MAVLINK LOOP =================
async def mavlink_loop():
    global last_packet_time, connected

    while True:
        if master is None or not connected:
            reset_connection()
            connect_mavlink()
            await asyncio.sleep(RECONNECT_DELAY)
            continue

        got_packet = False

        # Drain ALL packets
        while True:
            msg = master.recv_match(blocking=False)
            if not msg:
                break

            got_packet = True
            last_packet_time = time.time()

            t = msg.get_type()

            if t == "ATTITUDE":
                telemetry["roll"]  = round(msg.roll  * 57.2958, 2)
                telemetry["pitch"] = round(msg.pitch * 57.2958, 2)
                telemetry["yaw"]   = round(msg.yaw   * 57.2958, 2)

            elif t == "RC_CHANNELS":
                telemetry["throttle"] = int(msg.chan3_raw)

            elif t == "SYS_STATUS":
                if msg.voltage_battery > 0:
                    telemetry["battery"] = round(msg.voltage_battery / 1000, 2)

            elif t == "GLOBAL_POSITION_INT":
                telemetry["lat"] = round(msg.lat / 1e7, 6)
                telemetry["lng"] = round(msg.lon / 1e7, 6)

        now = time.time()

        if got_packet:
            telemetry["heartbeat"] = True
        elif last_packet_time and (now - last_packet_time) > HEARTBEAT_TIMEOUT:
            print("⚠️ Heartbeat timeout — hard reset")
            reset_connection()

        await asyncio.sleep(0)

# ================= BROADCAST =================
async def broadcast_loop():
    while True:
        telemetry["tick"] += 1

        if telemetry["lat"] and telemetry["lng"]:
            telemetry["event_mode"] = "MISSING_CHILD"
            telemetry["child_confidence"] = 0.87
            telemetry["operator_note"] = "Child last seen near ghat area"
            telemetry["last_seen_age_sec"] = telemetry["tick"] / BROADCAST_HZ
            telemetry["search_radius_m"] = int(
                telemetry["last_seen_age_sec"] * 1.2
            )

        payload = json.dumps(telemetry)

        if clients:
            await asyncio.gather(
                *(c.send(payload) for c in list(clients)),
                return_exceptions=True
            )

        await asyncio.sleep(1 / BROADCAST_HZ)

# ================= WEBSOCKET =================
async def ws_handler(ws):
    print("🌐 Web client connected")
    clients.add(ws)
    try:
        async for _ in ws:
            pass
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
