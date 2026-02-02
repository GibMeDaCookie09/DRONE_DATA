document.addEventListener("DOMContentLoaded", () => {

  // DOM ELEMENTS
 
  const roll = document.getElementById("roll");
  const pitch = document.getElementById("pitch");
  const yaw = document.getElementById("yaw");
  const throttle = document.getElementById("throttle");
  const voltage = document.getElementById("voltage");
  const commPath = document.getElementById("commPath");
  const lat = document.getElementById("lat");
  const lng = document.getElementById("lng");
  const logBox = document.getElementById("logBox");
  const video = document.getElementById("cameraFeed");
  const armStatus = document.getElementById("armStatus");
  const flightMode = document.getElementById("flightMode");
  const motorsEl = document.getElementById("motors");

  // Motor PWM
  const m1 = document.getElementById("m1");
  const m2 = document.getElementById("m2");
  const m3 = document.getElementById("m3");
  const m4 = document.getElementById("m4");

  if (!logBox) return;

  // INIT LOG
  logBox.innerHTML = "[SYSTEM] Log initialized<br>";

  // WEBSOCKET
  const ws = new WebSocket("ws://localhost:8765");

  ws.onopen = () => {
    logBox.innerHTML += "[SYSTEM] WebSocket connected<br>";
    commPath.innerText = "Connected via Wi-Fi";
    logBox.scrollTop = logBox.scrollHeight;
  };

  ws.onerror = () => {
    logBox.innerHTML += "[SYSTEM] WebSocket ERROR<br>";
  };

  // MAP
  let map = L.map("map").setView([18.5204, 73.8567], 18);

  let mapLayer = L.tileLayer(
    "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    { maxZoom: 20 }
  ).addTo(map);
  
  let droneMarker = L.marker([18.5204, 73.8567]).addTo(map);

  let satLayer = L.tileLayer(
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 20 }
  );

  let marker = L.marker([18.5204, 73.8567]).addTo(map);
  let isSatellite = false;

  // TELEMETRY
  let lastLogTime = 0;
  const LOG_INTERVAL = 1000;

  ws.onmessage = (event) => {
    const d = JSON.parse(event.data);

    //     Orientation    
    if (d.roll !== undefined) roll.innerText = `${d.roll}°`;
    if (d.pitch !== undefined) pitch.innerText = `${d.pitch}°`;
    if (d.yaw !== undefined) yaw.innerText = `${d.yaw}°`;

    //     Battery    
    if (d.battery !== undefined)
      voltage.innerText = `${d.battery} V`;

    //     Armed    
    if (d.armed) {
      armStatus.innerText = "ARMED";
      armStatus.className = "status armed";
    } else {
      armStatus.innerText = "DISARMED";
      armStatus.className = "status disarmed";
    }

    //     Flight mode    
    if (d.mode) flightMode.innerText = d.mode;

    //     Motor PWM    
    if (d.motor_pwm) {
      m1.innerText = d.motor_pwm.m1;
      m2.innerText = d.motor_pwm.m2;
      m3.innerText = d.motor_pwm.m3;
      m4.innerText = d.motor_pwm.m4;

      //  Throttle derived from motors (REAL)
      const avgThrottle = Math.round(
        (d.motor_pwm.m1 +
         d.motor_pwm.m2 +
         d.motor_pwm.m3 +
         d.motor_pwm.m4) / 4
      );
      throttle.innerText = `${avgThrottle} PWM`;

      //  Motor state
      const spinning = d.armed && avgThrottle > 1050;

      if (!d.armed) {
        motorsEl.innerText = "Stopped";
        motorsEl.style.color = "#dc2626";
      } else if (spinning) {
        motorsEl.innerText = "Spinning";
        motorsEl.style.color = "#22c55e";
      } else {
        motorsEl.innerText = "Idle";
        motorsEl.style.color = "#f59e0b";
      }
    }

    //     GPS    
    // LIVE GPS MARKER UPDATE
if (d.lat !== null && d.lng !== null) {
  const latitude = d.lat;
  const longitude = d.lng;

  lat.innerText = latitude.toFixed(6);
  lng.innerText = longitude.toFixed(6);

  // Move marker
  droneMarker.setLatLng([latitude, longitude]);

  // Follow drone
  map.setView([latitude, longitude], map.getZoom(), {
    animate: true
  });
}


    //     Heartbeat    
    const hbDot = document.querySelector(".top-left .dot");
    const hbLabel = document.querySelector(".top-left .label");

    if (d.heartbeat) {
      hbDot.style.background = "#22c55e";
      hbLabel.innerText = "Heartbeat OK";
    } else {
      hbDot.style.background = "#dc2626";
      hbLabel.innerText = "Heartbeat LOST";
    }

    //     Logging    
    const now = Date.now();
    if (now - lastLogTime > LOG_INTERVAL) {
      logBox.innerHTML +=
        `[${new Date().toLocaleTimeString()}] ` +
        `Roll:${d.roll ?? "-"} ` +
        `Pitch:${d.pitch ?? "-"} ` +
        `Yaw:${d.yaw ?? "-"} ` +
        `M1:${d.motor_pwm?.m1 ?? "-"} ` +
        `M2:${d.motor_pwm?.m2 ?? "-"} ` +
        `M3:${d.motor_pwm?.m3 ?? "-"} ` +
        `M4:${d.motor_pwm?.m4 ?? "-"}<br>`;
      logBox.scrollTop = logBox.scrollHeight;
      lastLogTime = now;
    }
  };

  // MAP TOGGLE
  window.toggleMapMode = () => {
    if (isSatellite) {
      map.removeLayer(satLayer);
      map.addLayer(mapLayer);
    } else {
      map.removeLayer(mapLayer);
      map.addLayer(satLayer);
    }
    isSatellite = !isSatellite;
  };

  // CAMERA
// ================= CAMERA =================
// ================= CAMERA =================
let camStream = null;
const videoContainer = document.querySelector(".video-container");

window.startCamera = async () => {
  if (camStream) return;

  // Force permission
  await navigator.mediaDevices.getUserMedia({ video: true });

  const devices = await navigator.mediaDevices.enumerateDevices();
  const videoDevices = devices.filter(d => d.kind === "videoinput");

  const fpvCam = videoDevices.find(d =>
    (d.label || "").toLowerCase().includes("usb") ||
    (d.label || "").toLowerCase().includes("pc camera") ||
    (d.label || "").toLowerCase().includes("skydroid")
  );

  if (!fpvCam) {
    alert("FPV receiver (USB camera) not found");
    return;
  }

  camStream = await navigator.mediaDevices.getUserMedia({
    video: { deviceId: { exact: fpvCam.deviceId } },
    audio: false
  });

  video.srcObject = camStream;
  video.muted = true;
  await video.play();
};

window.stopCamera = () => {
  if (!camStream) return;

  camStream.getTracks().forEach(track => track.stop());
  camStream = null;

  video.pause();
  video.srcObject = null;
  video.load();
};

window.toggleFullscreen = () => {
  const elem = videoContainer;

  if (!document.fullscreenElement) {
    elem.requestFullscreen?.() ||
    elem.webkitRequestFullscreen?.() ||
    elem.msRequestFullscreen?.();
  } else {
    document.exitFullscreen?.() ||
    document.webkitExitFullscreen?.() ||
    document.msExitFullscreen?.();
  }
};


  // COMMAND SENDER
  function sendCommand(cmd) {
    if (ws.readyState !== WebSocket.OPEN) {
      logBox.innerHTML += "[WARN] WebSocket not ready<br>";
      return;
    }
    ws.send(JSON.stringify({ command: cmd }));
    logBox.innerHTML += `[CMD] ${cmd}<br>`;
    logBox.scrollTop = logBox.scrollHeight;
  }

  
  // BUTTON ACTIONS
  window.arm = () => sendCommand("ARM");
  window.disarm = () => sendCommand("DISARM");
  window.takeoff = () => sendCommand("TAKEOFF");
  window.land = () => sendCommand("LAND");
  window.hold = () => sendCommand("HOLD");
  window.emergency = () => sendCommand("EMERGENCY");

});
