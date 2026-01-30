from pymavlink import mavutil
import asyncio
import websockets
import json
import time
import cv2
from aiohttp import web

# =========================
# CONFIG
# =========================
MAVLINK_CONN = 'COM8'
BAUDRATE = 115200

WS_HOST = "0.0.0.0"
WS_PORT = 8765
HTTP_PORT = 8080

# =========================
# MAVLINK CONNECT
# =========================
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUDRATE)
master.wait_heartbeat()
print("✅ MAVLink connected")

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

request(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30)   # Attitude
request(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)
request(mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
request(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)

# =========================
# TELEMETRY CACHE
# =========================
telemetry = {
    "roll": 0,
    "pitch": 0,
    "yaw": 0,
    "throttle": 0,
    "battery": 0,
    "lat": None,
    "lng": None,
    "path": "USB"
}

clients = set()

# =========================
# MAVLINK LOOP
# =========================
async def mavloop():
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
                    telemetry["battery"] = msg.voltage_battery / 1000

            elif t == "GLOBAL_POSITION_INT":
                telemetry["lat"] = msg.lat / 1e7
                telemetry["lng"] = msg.lon / 1e7

        await asyncio.sleep(0)

# =========================
# WEBSOCKET LOOP
# =========================
async def ws_loop():
    while True:
        if clients:
            data = json.dumps(telemetry)
            await asyncio.gather(*[c.send(data) for c in clients])
        await asyncio.sleep(1/60)

async def ws_handler(ws):
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.remove(ws)

# =========================
# CAMERA STREAM (LAPTOP)
# =========================
cap = cv2.VideoCapture(0)

async def camera_feed(request):
    async def stream(resp):
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            _, jpg = cv2.imencode('.jpg', frame)
            await resp.write(
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpg.tobytes() + b"\r\n"
            )
            await asyncio.sleep(0)

    resp = web.StreamResponse(
        status=200,
        headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
    )
    await resp.prepare(request)
    await stream(resp)
    return resp

# =========================
# MAIN
# =========================
async def main():
    ws_server = websockets.serve(ws_handler, WS_HOST, WS_PORT)

    app = web.Application()
    app.router.add_get('/camera', camera_feed)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", HTTP_PORT)

    await asyncio.gather(
        ws_server,
        site.start(),
        mavloop(),
        ws_loop()
    )

asyncio.run(main())
