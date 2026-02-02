import serial
import struct
import asyncio
import websockets
import json
import time

# ================= CONFIG =================
SERIAL_PORT = "COM10"        # <-- CONFIRM IN DEVICE MANAGER
BAUDRATE = 115200

WS_HOST = "0.0.0.0"
WS_PORT = 8765
BROADCAST_HZ = 60

# =============== MSP CONSTANTS ============
MSP_ATTITUDE = 108
MSP_ANALOG   = 110
MSP_STATUS   = 101
MSP_RAW_GPS  = 106
MSP_MOTOR    = 104

MSP_HZ = 120
MSP_INTERVAL = 1 / MSP_HZ

# =============== STATE ====================
clients = set()
msp_buffer = bytearray()

last_yaw = None   # ✅ REQUIRED (GLOBAL STATE)

telemetry = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,          # yaw rate (deg/sec)
    "battery": 0.0,
    "armed": False,
    "mode": "UNKNOWN",
    "lat": None,
    "lng": None,
    "motor_pwm": {"m1": 1000, "m2": 1000, "m3": 1000, "m4": 1000},
    "heartbeat": True,
    "tick": 0
}

# =============== MSP HELPERS ==============
def msp_request(cmd):
    size = 0
    checksum = size ^ cmd
    return bytes([36, 77, 60, size, cmd, checksum])  # $ M <

def read_msp_packet(ser):
    global msp_buffer

    data = ser.read(256)
    if data:
        msp_buffer.extend(data)

    while True:
        if len(msp_buffer) < 6:
            return None

        # Look for "$M>"
        if msp_buffer[0] != 36 or msp_buffer[1] != 77 or msp_buffer[2] != 62:
            msp_buffer.pop(0)
            continue

        size = msp_buffer[3]
        total_len = 6 + size

        if len(msp_buffer) < total_len:
            return None

        packet = msp_buffer[:total_len]
        del msp_buffer[:total_len]

        cmd = packet[4]
        payload = packet[5:5 + size]
        checksum = packet[-1]

        calc = size ^ cmd
        for b in payload:
            calc ^= b

        if calc != checksum:
            continue

        return cmd, payload

# =============== MSP LOOP =================
async def msp_loop():
    global last_yaw

    ser = serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        timeout=0,
        write_timeout=0
    )

    print("✅ Connected to Betaflight via MSP")

    last_request = 0

    while True:
        now = time.perf_counter()

        # ---- Request MSP data ----
        if now - last_request >= MSP_INTERVAL:
            ser.write(msp_request(MSP_ATTITUDE))
            ser.write(msp_request(MSP_ANALOG))
            ser.write(msp_request(MSP_STATUS))
            ser.write(msp_request(MSP_RAW_GPS))
            ser.write(msp_request(MSP_MOTOR))
            last_request = now

        pkt = read_msp_packet(ser)
        if pkt:
            cmd, data = pkt

            # -------- ATTITUDE --------
            if cmd == MSP_ATTITUDE:
                roll, pitch, yaw = struct.unpack("<hhh", data)

                roll  = roll / 10.0
                pitch = pitch / 10.0
                yaw   = yaw / 10.0  # absolute heading (0–360)

                telemetry["roll"]  = round(roll, 2)
                telemetry["pitch"] = round(pitch, 2)
                if yaw <=360:   
                    telemetry["yaw"]   = round(yaw*10, 2)

            # -------- BATTERY --------
            elif cmd == MSP_ANALOG:
                telemetry["battery"] = round(data[0] / 10.0, 2)

            # -------- STATUS --------
            elif cmd == MSP_STATUS:
                flags = struct.unpack("<H", data[6:8])[0]
                telemetry["armed"] = bool(flags & 1)
                telemetry["mode"] = "ARMED" if telemetry["armed"] else "DISARMED"

            # -------- GPS --------
            elif cmd == MSP_RAW_GPS and data[0]:
                telemetry["lat"] = struct.unpack("<i", data[1:5])[0] / 1e7
                telemetry["lng"] = struct.unpack("<i", data[5:9])[0] / 1e7

            # -------- MOTORS --------
            elif cmd == MSP_MOTOR:
                m1, m2, m3, m4 = struct.unpack("<4H", data[:8])
                telemetry["motor_pwm"].update({
                    "m1": m1,
                    "m2": m2,
                    "m3": m3,
                    "m4": m4
                })

        await asyncio.sleep(0.001)

# =============== WEBSOCKET ================
async def ws_handler(ws):
    clients.add(ws)
    try:
        async for _ in ws:
            pass
    finally:
        clients.discard(ws)

async def broadcast_loop():
    interval = 1 / BROADCAST_HZ
    while True:
        telemetry["tick"] += 1
        payload = json.dumps(telemetry)

        dead = set()
        for c in clients:
            try:
                await c.send(payload)
            except:
                dead.add(c)

        for c in dead:
            clients.discard(c)

        await asyncio.sleep(interval)

# ================= MAIN ===================
async def main():
    print(f"🌐 WebSocket running at ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.gather(
            msp_loop(),
            broadcast_loop()
        )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("🛑 Clean shutdown")
