// modified code by yoyostark
//things need to check
//1. time intervals
//2. coding refractoring check again
//3. what features to support 
//4. wifi essentials
//5. monitor prints
//6. pins number
//7. button_1 reset manually
//8. wss or ws for port and url
//9. vendor name and firmare version - in another words "optional things"
//10. should we send energy in WATT or KILO_WATT
//11. the location in the payload frame 
//12. OVER_CURRENT_LIMIT


#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <lwip/apps/sntp.h> 

/************************ WiFi ************************/ 
  //UPDATE THESE
  const char* ssid = "WE0EBA62";
  const char* password = "aa028759";


/************************ OCPP Server ************************/ 
  const char* ocpp_server = "portal.elsewedyplug.com";          // change this back to portal.elsewedyplug.com
  const int ocpp_port = 443;                         // and this to 443
  const char* ocpp_path = "/ocpp/DA_SMARTPLUG_03";
  const char* charge_point_id = "DA_SMARTPLUG_03";
  WebSocketsClient webSocket;
  bool bootAccepted = false;

/************************ Pins ************************/
  #define RELAY_PIN        4
  #define BUTTON_PIN       18
  #define BUTTON1_PIN      21
  #define RS485_DIR_PIN    14
  #define CF_PIN           34
  #define CF1_PIN          35
  #define SEL_PIN          22

/************************ Calibration & Limits ************************/
  #define OVER_CURRENT_LIMIT   45.0
  #define POWER_CALIBRATION    1.34840
  #define VOLT_CALIBRATION     0.125218
  #define CURR_CALIBRATION     0.011324
  #define DEBOUNCE_DELAY       500
  #define SEC_PER_HOUR         3600
  #define WATT_TO_KWH          1000

/************************ OCPP Message Intervals*/
  #define HEARTBEAT_INTERVAL 20000
  #define STATUS_INTERVAL    10000
  #define METER_INTERVAL     15000

/************************ Runtime Variables ************************/
  volatile unsigned long cfCount = 0, cf1Count = 0;
  double voltage = 0, current = 0, power = 0, energy = 0, energy_K_W_H =0;
  bool relayState = true;
  bool overcurrentTripped = false;
  unsigned long message_counter = 1;
  volatile bool buttonPressed = false;
  unsigned long lastDebounceTime = 0;
  DynamicJsonDocument responsePayload(1024);
  int currentTransactionId = -1;


  enum SensorReadState { READ_VOLTAGE_PREP, READ_VOLTAGE_COUNT, READ_CURRENT_PREP, READ_CURRENT_COUNT, READ_DONE };
  SensorReadState sensorState = READ_VOLTAGE_PREP;

  unsigned long sensorTimer = 0;
  const unsigned long settleTime = 50;
  const unsigned long countTime = 450;

  double tempVoltageCount = 0;
  double tempCurrentCount = 0;


/************************ RS485 ************************/ 
  void measurment_serial_sending(double power,double voltage,double current, double energy)
  {
    Serial.print("V:"); 
    Serial.print(voltage,1);
    Serial.print(" I:");
    Serial.print(current,4);
    Serial.print(" P:");
    Serial.print(power,1);
    Serial.print(" E:");
    Serial.println(energy,5);
    Serial.flush();
    digitalWrite(RS485_DIR_PIN,LOW);
    delay(100);

  }

/************************ Timestamp ************************/
  String getCurrentTimestamp() 
  {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) 
    {
      return "1970-01-01T00:00:00Z";  // fallback
    }
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
  }

/************************ Interrupts ************************/
  void IRAM_ATTR cfISR() { cfCount++; }
  void IRAM_ATTR cf1ISR() { cf1Count++; }

  void IRAM_ATTR buttonISR() 
  {
    buttonPressed = true;
  }

  void handleChangeAvailability(String messageId, JsonObject payload);
  void handleReset(String messageId, JsonObject payload);
  void handleDataTransfer(String messageId, JsonObject payload);
  //void handleGetVariables(String messageId, JsonObject payload);

/************************ Setup ************************/
void setup() 
{
  Serial.begin(115200);
  Serial.println("Chip started");

  pinMode(RELAY_PIN, OUTPUT);
 //  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RS485_DIR_PIN, OUTPUT);
  pinMode(CF_PIN, INPUT);
  pinMode(CF1_PIN, INPUT);
  pinMode(SEL_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(RS485_DIR_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(CF_PIN), cfISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(CF1_PIN), cf1ISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500); Serial.print(".");
  }
  Serial.println(F("\nWiFi connected!"));
  
 //configuring the NTP for real date and time  
  configTzTime("UTC", "pool.ntp.org", "time.nist.gov");
  Serial.println(F("Waiting for NTP time sync..."));

  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) 
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println(F("\nNTP time synchronized!"));

 //init websocket

  webSocket.beginSSL(ocpp_server, ocpp_port, ocpp_path);             //change this back to beginssl
  webSocket.setExtraHeaders("Sec-WebSocket-Protocol: ocpp1.6");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

/************************ OCPP Sending ************************/
  void sendBootNotification();
  void sendStatusNotification();
  void sendHeartbeat();
  void sendMeterValues();
  void startLocalTransaction();


    void handleRemoteStopTransaction(String messageId, JsonObject payload) 
  {
    Serial.println("RemoteStopTransaction received");

    // Send OCPP response
    DynamicJsonDocument res(512);
    res[0] = 3;
    res[1] = messageId;
    JsonObject resPayload = res.createNestedObject(2);
    resPayload["status"] = "Accepted";

    String response;
    serializeJson(res, response);
    webSocket.sendTXT(response);
    Serial.println("Responded to RemoteStopTransaction");

    // Turn OFF the relay
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
  }

void handleRemoteStartTransaction(String messageId, JsonObject payload) 
{
    Serial.println("RemoteStartTransaction received");
    Serial.print("Incoming messageId: ");
    Serial.println(messageId);

    // Prepare OCPP CALLRESULT response
    DynamicJsonDocument res(512);
    res[0] = 3; // CALLRESULT
    res[1] = messageId; // must match server's ID exactly
    JsonObject resPayload = res.createNestedObject(2);
    resPayload["status"] = "Accepted";

    String response;
    serializeJson(res, response);

    // DEBUG: print exactly what's being sent
    Serial.print("Sending CALLRESULT: ");
    Serial.println(response);

    // Send back to server
    webSocket.sendTXT(response);
    Serial.println("Responded to RemoteStartTransaction");

    // Turn ON relay
    digitalWrite(RELAY_PIN, HIGH);
    relayState = true;

    // Optionally tell server status changed right now
    sendStatusNotification();
}

void sendStartTransaction() 
{
    DynamicJsonDocument doc(512);
    doc[0] = 2;
    doc[1] = "startTx_" + String(message_counter++);
    doc[2] = "StartTransaction";

    JsonObject p = doc.createNestedObject(3);
    p["connectorId"] = 1;
    p["idTag"] = "LOCAL";
    p["meterStart"] = (int)energy;  // Wh
    p["timestamp"] = getCurrentTimestamp();

    String msg; 
    serializeJson(doc, msg);
    webSocket.sendTXT(msg);
    Serial.println("[OCPP] Sent StartTransaction");
}


  void webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
  {
    switch (type)
    {
      case WStype_DISCONNECTED:
        Serial.println("WebSocket Disconnected");
        break;

      case WStype_CONNECTED:
        Serial.printf("WebSocket Connected: %s\n", payload);
        sendBootNotification();
        break;

      case WStype_TEXT: {
        Serial.printf("[WS] RX: %s\n", payload);
        DynamicJsonDocument doc(4096); // increase input buffer
        DeserializationError err = deserializeJson(doc, payload, length);
        if (err) { Serial.printf("[JSON] Parse error: %s\n", err.c_str()); return; }

        int messageType = doc[0] | 0;        // 2=CALL, 3=CALLRESULT
        String messageId = doc[1] | "";      // safe even if server sends number

        if (messageType == 3) { handleCallResult(messageId , doc[2]); return; }
        if (messageType == 2) {
          String action = doc[2] | "";
          JsonObject payloadObj = doc[3].as<JsonObject>();
          handleCall(action, messageId, payloadObj);
          return;
        }
        Serial.println("[OCPP] Unhandled message type");
      } break;

      default:
        break;
    }
  }


void handleCallResult(const String& messageId, JsonVariant payload)
{
    Serial.print(F("Received CALLRESULT for messageId: "));
    Serial.println(messageId);

    if (messageId.startsWith("boot_"))
    { 
      bootAccepted = true;
     sendStartTransaction();
    }
    else if (messageId.startsWith("heart_")) Serial.println(F("Heartbeat response received."));
    else if (messageId.startsWith("mv_")) Serial.println(F("MeterValues response received."));
    else if (messageId.startsWith("status_")) Serial.println(F("StatusNotification response received."));
    else if (messageId.startsWith("dt_")) Serial.println(F("DataTransfer response received."));
    else if (messageId.startsWith("reset_")) Serial.println(F("Reset response received."));
    else if (messageId.startsWith("startTx_")) 
    {
        Serial.println(F("StartTransaction response received."));
        // Extract transactionId from payload if available
        if (!payload.isNull())
        {
            if (payload.containsKey("transactionId"))
            {
                currentTransactionId = payload["transactionId"].as<int>();
                Serial.print("Stored transactionId: ");
                Serial.println(currentTransactionId);
            }
            else
            {
                Serial.println("Warning: StartTransaction response missing transactionId");
            }
        }
        else
        {
            Serial.println("Warning: StartTransaction response payload is empty");
        }
    }
    //else if (messageId.startsWith("getvar_")) Serial.println(F("GetVariables response received."));
    else Serial.println(F("Unrecognized CALLRESULT messageId."));
}

void handleCall(const String& action, const String& messageId, JsonObject payloadObj)
{
    if (action == "ChangeAvailability") handleChangeAvailability(messageId, payloadObj);
    else if (action == "Reset") handleReset(messageId, payloadObj);
    else if (action == "DataTransfer") handleDataTransfer(messageId, payloadObj);
    //else if (action == "GetVariables") handleGetVariables(messageId, payloadObj);
    else if (action == "RemoteStartTransaction")
    {
       handleRemoteStartTransaction(messageId, payloadObj);
      Serial.println("RemoteStartTransaction received from server");
    }
    else if (action == "RemoteStopTransaction") handleRemoteStopTransaction(messageId, payloadObj);
    else Serial.println("Unhandled OCPP CALL action: " + action);
}

void handleChangeAvailability(String messageId, JsonObject payload)
  {
    String status = payload["type"];
    bool accepted = false;
    Serial.print("ChangeAvailability command received: ");
    Serial.println(status);

    if (status == "Operative") 
    {
      digitalWrite(RELAY_PIN, HIGH);
      relayState = true;
      accepted = true;
      Serial.println("Relay ON by ChangeAvailability");
    }
     else if (status == "Inoperative") 
    {
      digitalWrite(RELAY_PIN, LOW);
      relayState = false;
      accepted = true;
      Serial.println("Relay OFF by ChangeAvailability");
    }
    else 
    {
      Serial.println("[OCPP] Unknown ChangeAvailability type");
    }

    DynamicJsonDocument res(512);
    res[0] = 3;
    res[1] = messageId;
    JsonObject resPayload = res.createNestedObject(2);
    resPayload["status"] = accepted ? "Accepted" : "Rejected";
    String response;
    serializeJson(res, response);
    webSocket.sendTXT(response);
    Serial.println(F("Responded to ChangeAvailability."));
  }

void handleReset(String messageId, JsonObject payload)
  {
    String resetType = payload["type"];
    Serial.print("Reset command received: ");
    Serial.println(resetType);
    DynamicJsonDocument res(512);
    res[0] = 3;
    res[1] = messageId;
    JsonObject resPayload = res.createNestedObject(2);
    resPayload["status"] = "Accepted";

    String response;
    serializeJson(res, response);
    webSocket.sendTXT(response);
    Serial.println(F("Responded to Reset with Accepted."));
    if (resetType == "Immediate") 
    {
      delay(1000);
      ESP.restart();
    }
  }

void handleDataTransfer(String messageId, JsonObject payload)
  {
    String vendorId = payload["vendorId"] | "";
    String msgId = payload["messageId"] | "";
    String data = payload["data"] | "";
    Serial.println(F("DataTransfer received:"));
    Serial.printf("  VendorId: %s\n", vendorId.c_str());
    Serial.printf("  MessageId: %s\n", msgId.c_str());
    Serial.printf("  Data: %s\n", data.c_str());

    DynamicJsonDocument res(1024);
    res[0] = 3;
    res[1] = messageId;
    JsonObject resPayload = res.createNestedObject(2);
    resPayload["status"] = "Accepted";
    resPayload["data"] = String("Echo: ") + data;

    String response;
    serializeJson(res, response);
    webSocket.sendTXT(response);
    Serial.println(F("Responded to DataTransfer."));
  }

void sendBootNotification() 
  {
    DynamicJsonDocument doc(512);
    doc[0] = 2;
    doc[1] = "boot_" + String(message_counter++);
    doc[2] = "BootNotification";

    JsonObject p = doc.createNestedObject(3);
    p["chargePointVendor"] = "Dezign Arena";
    p["chargePointModel"]  = "DA_OCPP_v2";
    p["firmwareVersion"]   = "v1.1.0";
    p["chargePointSerialNumber"] = charge_point_id; // optional fields ok

    p["reason"] = "PowerUp";

    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
    Serial.println(F("Sent BootNotification (1.6)"));
  }


void sendStatusNotification() 
{
  DynamicJsonDocument doc(512);
  doc[0] = 2;
  doc[1] = "status_" + String(message_counter++);
  doc[2] = "StatusNotification";
  JsonObject p = doc.createNestedObject(3);
  p["timestamp"] = getCurrentTimestamp();

  // map your internal state -> OCPP1.6 status values
  String status;
  if (overcurrentTripped)           status = "Faulted";
  else if (!relayState)             status = "Available";
  else if (relayState && current >= 0.10) status = "Charging";
  else                              status = "Reserved";

  p["connectorId"] = 1;          // 1.6 uses connectorId (int)
  p["errorCode"]   = "NoError";  // required field in 1.6
  p["status"]      = status;

  String msg; serializeJson(doc, msg);
  webSocket.sendTXT(msg);
  Serial.printf("[OCPP] Status sent: %s\n", status.c_str());
}

void sendHeartbeat() 
  {
    DynamicJsonDocument doc(256);
    doc[0] = 2;
    doc[1] = "heart_" + String(message_counter++);
    doc[2] = "Heartbeat";
    doc.createNestedObject(3);
    String msg;
    serializeJson(doc, msg);
    webSocket.sendTXT(msg);
  }

void sendMeterValues() 
{
  DynamicJsonDocument doc(1024);
  doc[0] = 2;
  doc[1] = "mv_" + String(message_counter++);
  doc[2] = "MeterValues";

  JsonObject payload = doc.createNestedObject(3);
  payload["connectorId"] = 1;
  // payload["transactionId"] = ... if you are inside a transaction

  JsonArray meterValue = payload.createNestedArray("meterValue");
  JsonObject entry = meterValue.createNestedObject();
  entry["timestamp"] = getCurrentTimestamp();

  JsonArray sampled = entry.createNestedArray("sampledValue");

  // Voltage
  {
    JsonObject v = sampled.createNestedObject();
    v["value"] = voltage;
    v["measurand"] = "Voltage";
    v["context"] = "Sample.Periodic";
    v["location"] = "Outlet";
    v["unit"] = "V";
  }

  // Current
  {
    JsonObject a = sampled.createNestedObject();
    a["value"] = current;
    a["measurand"] = "Current.Import";
    a["context"] = "Sample.Periodic";
    a["location"] = "Outlet";
    a["unit"] = "A";
  }

  // Power
  {
    JsonObject p = sampled.createNestedObject();
    p["value"] = power;
    p["measurand"] = "Power.Active.Import";
    p["context"] = "Sample.Periodic";
    p["location"] = "Outlet";
    p["unit"] = "W";
  }

  // Energy (Wh)
  {
    JsonObject e = sampled.createNestedObject();
    e["value"] = energy;
    e["measurand"] = "Energy.Active.Import.Register";
    e["context"] = "Sample.Periodic";
    e["location"] = "Outlet";
    e["unit"] = "Wh";
  }

  String msg; serializeJson(doc, msg);
  webSocket.sendTXT(msg);
  Serial.printf("[METER] Sent MeterValues V=%.1f I=%.2f P=%.1f E=%.3fWh\n", voltage, current, power, energy);
}



/************************ Sensor Reading ************************/
void readSensorValues() 
{
  // Read Voltage
    digitalWrite(SEL_PIN, HIGH);
    delay(50);  // short settle time
    cf1Count = 0;
    delay(450);  // count voltage pulses
    double cf1CountVoltage = 2 * cf1Count;

  // Read Current
    digitalWrite(SEL_PIN, LOW);
    delay(50);
    cf1Count = 0;
    delay(450);  // count current pulses
    double cf1CountCurrent = 2 * cf1Count;

  // Calculate values
  voltage = cf1CountVoltage * VOLT_CALIBRATION;
  current = cf1CountCurrent * CURR_CALIBRATION;
  power = cfCount * POWER_CALIBRATION;
  energy += power / SEC_PER_HOUR;
  energy_K_W_H = energy / WATT_TO_KWH;

  // Reset counters
  cfCount = 0;
  cf1Count = 0;
  
  // Overcurrent check
  if (current > OVER_CURRENT_LIMIT) 
  {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    overcurrentTripped = true;
  }
}

/************************ Updated sensor Reading ************************/
void updateSensorFSM() 
{
  switch (sensorState) 
  {
    case READ_VOLTAGE_PREP:
      digitalWrite(SEL_PIN, HIGH);
      cf1Count = 0;
      sensorTimer = millis();
      sensorState = READ_VOLTAGE_COUNT;
      break;

    case READ_VOLTAGE_COUNT:
      if (millis() - sensorTimer >= countTime) 
      {
        tempVoltageCount = (20 * cf1Count) / 9.0;
        sensorState = READ_CURRENT_PREP;
      }
      break;

    case READ_CURRENT_PREP:
      digitalWrite(SEL_PIN, LOW);
      cf1Count = 0;
      sensorTimer = millis();
      sensorState = READ_CURRENT_COUNT;
      break;

    case READ_CURRENT_COUNT:
      if (millis() - sensorTimer >= countTime) 
      {
        tempCurrentCount = (20 * cf1Count) / 9.0;
        sensorState = READ_DONE;
      }
      break;

    case READ_DONE:
      voltage = tempVoltageCount * VOLT_CALIBRATION;
      current = tempCurrentCount * CURR_CALIBRATION;
      power = cfCount * POWER_CALIBRATION;
      energy += power / SEC_PER_HOUR;
      energy_K_W_H = energy / WATT_TO_KWH;

      cfCount = 0;
      cf1Count = 0;

      if (current > OVER_CURRENT_LIMIT) 
      {
        digitalWrite(RELAY_PIN, LOW);
        relayState = false;
        overcurrentTripped = true;
      }

      digitalWrite(RS485_DIR_PIN, HIGH);
      delay(10);
      measurment_serial_sending(power, voltage, current, energy_K_W_H);

      sensorState = READ_VOLTAGE_PREP;  // Restart FSM
      break;
  }
}

  void checkButton(unsigned long now)
  {
    bool localPress = false;

    noInterrupts(); // disable briefly
    if (buttonPressed) 
    {
      localPress = true;
      buttonPressed = false;
    } 
    interrupts(); // re-enable

  if (localPress && (now - lastDebounceTime > DEBOUNCE_DELAY)) 
    {
      lastDebounceTime = now;
      relayState = !relayState;
      digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
      Serial.println(relayState ? "Relay ON" : "Relay OFF");
    }
  }


/************************ Loop ************************/
void loop() 
{
  //ArduinoOTA.handle();
  webSocket.loop();
  //readSensorValues();                     // in test cases replace it with updateDemoVariables();  // in real life use readSensorValues()
  updateDemoVariables();
  //updateSensorFSM();

  static unsigned long lastStatus = 0, lastHeartbeat = 0, lastMeter = 0;
  unsigned long now = millis();

  checkButton(now);

  if (bootAccepted) 
  {
    if (now - lastStatus > STATUS_INTERVAL) 
    {
      sendStatusNotification();
      lastStatus = now;
    }
    if (now - lastHeartbeat > HEARTBEAT_INTERVAL ) 
    {
      sendHeartbeat();
      lastHeartbeat = now;
    }
    if (now - lastMeter > METER_INTERVAL) 
    {
      sendMeterValues();
      lastMeter = now;
    }
  }
}

//end of code

void updateDemoVariables()
 {
  static unsigned long lastUpdateMillis = millis();

  // Simulate changing sensor values
  current = (random(8, 11) / 10.0);  // 0.8 to 1.1 A
  voltage = (random(2130, 2190) / 10.0); // 210.0V to 220.0V
  power = voltage * current; // Watts

  // Calculate elapsed time in seconds
  unsigned long now = millis();
  float elapsedSeconds = (now - lastUpdateMillis) / 1000.0;
  lastUpdateMillis = now;

  // Energy (in Wh) = Power (W) * time (h)
  energy += power * (elapsedSeconds / 3600.0); // accurate accumulation
}
