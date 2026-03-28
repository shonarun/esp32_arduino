#include "comms.h"
#include "config.h"
#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; background-color: #222; color: white;}
    .btn { display: block; width: 80%; margin: 20px auto; padding: 30px; font-size: 30px; font-weight: bold; border-radius: 10px; border: none; color: white; cursor: pointer; }
    .btn-arm { background-color: #28a745; }
    .btn-kill { background-color: #dc3545; padding: 50px; font-size: 40px;}
    .btn:active { opacity: 0.7; }
    #status { font-size: 20px; margin-top: 20px; font-weight: bold; color: #ffc107; }
  </style>
</head>
<body>
  <h1>LITEWING v1.2</h1>
  <div id="status">STATUS: AWAITING COMMAND</div>
  
  <button class="btn btn-kill" onclick="sendCommand('/kill', 'KILLED', '#dc3545')">EMERGENCY KILL</button>
  <button class="btn btn-arm" onclick="sendCommand('/arm', 'ARMED', '#28a745')">ARM DRONE</button>

  <script>
    function sendCommand(url, statusText, statusColor) {
      if (navigator.vibrate) navigator.vibrate(200); 
      fetch(url, { method: 'POST' })
        .then(response => {
          if (response.ok) {
            const statusEl = document.getElementById('status');
            statusEl.innerText = 'STATUS: ' + statusText;
            statusEl.style.color = statusColor;
          }
        })
        .catch(err => console.error('Command failed to send:', err));
    }
  </script>
</body>
</html>
)rawliteral";

void Task_Comms(void *pvParameters) {
  WiFi.softAP("LITEWING_DRONE", "12345678"); 
  
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", INDEX_HTML);
  });

  server.on("/arm", HTTP_POST, []() {
    emergency_kill = false;
    is_armed = true;
    Serial.println("WEB: DRONE ARMED!");
    server.send(200, "text/plain", "Armed");
  });

  server.on("/kill", HTTP_POST, []() {
    emergency_kill = true;
    is_armed = false;
    is_flying = false;
    Serial.println("WEB: EMERGENCY KILL ENGAGED!");
    ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
    ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
    server.send(200, "text/plain", "Killed");
  });

  server.begin();
  Serial.println("Web Server Started on 192.168.4.1");

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}