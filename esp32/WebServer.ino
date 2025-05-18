#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// SoftAP credentials
const char* ap_ssid = "ESP32-Motor";
const char* ap_password = "motor123"; // Optional. Set to "" for open network.

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
  } 
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } 
  else if (type == WS_EVT_DATA) {
    String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    Serial.print("Received speed command via WebSocket: ");
    Serial.println(msg);
    // In the future, parse msg and apply to motor
  }
}

void setup() {
  Serial.begin(115200);

  // Start WiFi in AP mode
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("ESP32 SoftAP IP address: ");
  Serial.println(IP);

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Optional: Serve a simple HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<h2>ESP32 WebSocket Server (AP Mode)</h2>");
  });

  server.begin();
  Serial.println("Web server started in AP mode.");
}

void loop() {
  // Nothing needed here for AsyncWebServer
}
