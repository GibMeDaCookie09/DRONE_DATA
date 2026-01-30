from pymavlink import mavutil
import asyncio, websockets, json, time

# ================= CONFIG =================
MAVLINK_UDP = "udp:0.0.0.0:14550"
WS_HOST = "0.0.0.0"
WS_PORT = 8765

HEARTBEAT_TIMEOUT = 3.0
RECONNECT_DELAY = 1.0
BROADCAST_HZ = 20

# ================= STATE =================
master = None
last_packet_time = None
clients = set()

telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "battery": 0.0,
    "lat": None,
    "lng": None,
    "heartbeat": False,
    "path": "WiFi",
    "armed": False,
    "mode": "DISCONNECTED",
    "throttle": 1000,   #  RC throttle (CH3)
    "motors_active": False,
    "tick": 0,
    "motor_pwm": {
        "m1": 1000,
        "m2": 1000,
        "m3": 1000,
        "m4": 1000
    }
}

# ================= CONNECT =================
def connect_mavlink():
    global master, last_packet_time
    try:
        print("🔄 Connecting MAVLink...")
        master = mavutil.mavlink_connection(MAVLINK_UDP)
        master.wait_heartbeat(timeout=5)
        last_packet_time = time.time()
        print("✅ MAVLink connected")

        # Request required streams (NO deprecated ones)
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1
        )  # ATTITUDE

        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
        )  # GPS

        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1
        )  # SYS_STATUS

        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1
        )  # RC INPUT (THROTTLE)

    except Exception as e:
        print("❌ MAVLink connect failed:", e)
        master = None

# ================= COMMANDS =================
def arm():
    if not master:
        return
    master.set_mode("GUIDED")
    time.sleep(0.3)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

def disarm():
    if not master:
        return
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def takeoff(alt=3):
    if not telemetry["armed"]:
        return
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, alt
    )

def land():
    if master:
        master.set_mode("LAND")

def emergency():
    if master:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0, 1, 0, 0, 0, 0, 0, 0
        )

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
            telemetry["heartbeat"] = True

            t = msg.get_type()

            # -------- HEARTBEAT --------
            if t == "HEARTBEAT":
                telemetry["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                telemetry["mode"] = mavutil.mode_string_v10(msg)

            # -------- ATTITUDE --------
            elif t == "ATTITUDE":
                telemetry["roll"]  = round(msg.roll * 57.2958, 2)
                telemetry["pitch"] = round(msg.pitch * 57.2958, 2)
                telemetry["yaw"]   = round(msg.yaw * 57.2958, 2)

            # -------- BATTERY --------
            elif t == "SYS_STATUS" and msg.voltage_battery > 0:
                telemetry["battery"] = round(msg.voltage_battery / 1000, 2)

            # -------- RC INPUT (THROTTLE) --------
            elif t == "RC_CHANNELS":
                telemetry["throttle"] = msg.chan3_raw  #  REAL throttle

            # -------- MOTOR OUTPUT --------
            elif t == "SERVO_OUTPUT_RAW":
                telemetry["motor_pwm"]["m1"] = msg.servo1_raw
                telemetry["motor_pwm"]["m2"] = msg.servo2_raw
                telemetry["motor_pwm"]["m3"] = msg.servo3_raw
                telemetry["motor_pwm"]["m4"] = msg.servo4_raw

                telemetry["motors_active"] = (
                    telemetry["armed"] and
                    max(
                        msg.servo1_raw,
                        msg.servo2_raw,
                        msg.servo3_raw,
                        msg.servo4_raw
                    ) > 1050
                )

            # -------- GPS --------
            elif t == "GLOBAL_POSITION_INT":
                telemetry["lat"] = msg.lat / 1e7
                telemetry["lng"] = msg.lon / 1e7

        # -------- TIMEOUT --------
        if last_packet_time and time.time() - last_packet_time > HEARTBEAT_TIMEOUT:
            telemetry["heartbeat"] = False
            telemetry["armed"] = False
            telemetry["motors_active"] = False
            telemetry["mode"] = "DISCONNECTED"
            master = None

        await asyncio.sleep(0)

# ================= WS =================
async def ws_handler(ws):
    clients.add(ws)
    try:
        async for msg in ws:
            cmd = json.loads(msg).get("command")
            if cmd == "ARM": arm()
            elif cmd == "DISARM": disarm()
            elif cmd == "TAKEOFF": takeoff()
            elif cmd == "LAND": land()
            elif cmd == "EMERGENCY": emergency()
    finally:
        clients.remove(ws)

# ================= BROADCAST =================
async def broadcast_loop():
    while True:
        telemetry["tick"] += 1
        if clients:
            payload = json.dumps(telemetry)
            await asyncio.gather(*(c.send(payload) for c in clients))
        await asyncio.sleep(1 / BROADCAST_HZ)

# ================= MAIN =================
async def main():
    print(f"🚀 WebSocket running at ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.gather(mavlink_loop(), broadcast_loop())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("🛑 Server stopped")

