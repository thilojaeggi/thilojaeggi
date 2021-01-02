#include <Arduino.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
void createName();
bool initBTSerial();
void readBTSerial();
void gotIP(system_event_id_t event);
void lostCon(system_event_id_t event);
void gotCon(system_event_id_t event);
void staStart(system_event_id_t event);
void staStop(system_event_id_t event);
void connectWiFi();
AsyncWebServer server(80);
const char *SOCKET = "socket";

/** Define GPIOs for Relais */
const byte relay_io1 = 21;
const byte relay_io2 = 19;
const byte relay_io3 = 18;
const byte relay_io4 = 5;
/** State for Relais */
bool relay1 = false;
bool relay2 = false;
bool relay3 = false;
bool relay4 = false;

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

/** Unique device name */
char apName[] = "ljus-xxxxxxxxxxxx";

/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;
bool ledOn = false;
bool btSerialOn = false;
/**
   Create unique device name from MAC address
 **/
void createName()
{
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Write unique name into apName
  sprintf(apName, "ljus-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

// SerialBT class
BluetoothSerial SerialBT;
#define ONBOARD_LED 2

/** Buffer for JSON string */
// MAx size is 51 bytes for frame:
// {"ssidPrim":"","pwPrim":"","ssidSec":"","pwSec":""}
// + 4 x 32 bytes for 2 SSID's and 2 passwords
StaticJsonDocument<200> jsonBuffer;

/**
   initBTSerial
   Initialize Bluetooth Serial
   Start BLE server and service advertising
   @return <code>bool</code>
        true if success
        false if error occured
*/
bool initBTSerial()
{
  btSerialOn = true;
  if (!SerialBT.begin(apName))
  {
    Serial.println("Failed to start BTSerial");
    return false;
  }
  Serial.println("BTSerial active. Device name: " + String(apName));
  return true;
}

/**
   readBTSerial
   read all data from BTSerial receive buffer
   parse data for valid WiFi credentials
*/
void readBTSerial()
{
  uint64_t startTimeOut = millis();
  String receivedData;
  int msgSize = 0;
  // Read RX buffer into String
  while (SerialBT.available() != 0)
  {
    receivedData += (char)SerialBT.read();
    msgSize++;
    // Check for timeout condition
    if ((millis() - startTimeOut) >= 5000)
      break;
  }
  SerialBT.flush();
  Serial.println("Received message " + receivedData + " over Bluetooth");

  // decode the message
  int keyIndex = 0;
  for (int index = 0; index < receivedData.length(); index++)
  {
    receivedData[index] = (char)receivedData[index] ^ (char)apName[keyIndex];
    keyIndex++;
    if (keyIndex >= strlen(apName))
      keyIndex = 0;
  }

  Serial.println("Received message " + receivedData + " over Bluetooth");

  /** Json object for incoming data */
  auto error = deserializeJson(jsonBuffer, receivedData);
  if (!error)
  {
    if (jsonBuffer.containsKey("ssidPrim") &&
        jsonBuffer.containsKey("pwPrim") &&
        jsonBuffer.containsKey("ssidSec") &&
        jsonBuffer.containsKey("pwSec"))
    {
      ssidPrim = jsonBuffer["ssidPrim"].as<String>();
      pwPrim = jsonBuffer["pwPrim"].as<String>();
      ssidSec = jsonBuffer["ssidSec"].as<String>();
      pwSec = jsonBuffer["pwSec"].as<String>();
      Preferences preferences;
      preferences.begin("WiFiCred", false);
      preferences.putString("ssidPrim", ssidPrim);
      preferences.putString("ssidSec", ssidSec);
      preferences.putString("pwPrim", pwPrim);
      preferences.putString("pwSec", pwSec);
      preferences.putBool("valid", true);
      preferences.end();

      Serial.println("Received over bluetooth:");
      Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
      Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
      connStatusChanged = true;
      hasCredentials = true;
    }
    else if (jsonBuffer.containsKey("erase"))
    { // {"erase":"true"}
      Serial.println("Received erase command");
      Preferences preferences;
      preferences.begin("WiFiCred", false);
      preferences.clear();
      preferences.end();
      connStatusChanged = true;
      hasCredentials = false;
      ssidPrim = "";
      pwPrim = "";
      ssidSec = "";
      pwSec = "";

      int err;
      err = nvs_flash_init();
      Serial.println("nvs_flash_init: " + err);
      err = nvs_flash_erase();
      Serial.println("nvs_flash_erase: " + err);
    }
    else if (jsonBuffer.containsKey("read"))
    { // {"read":"true"}
      Serial.println("BTSerial read request");
      String wifiCredentials;
      jsonBuffer.clear();

      /** Json object for outgoing data */
      jsonBuffer.clear();
      jsonBuffer["ssidPrim"] = ssidPrim;
      jsonBuffer["pwPrim"] = pwPrim;
      jsonBuffer["ssidSec"] = ssidSec;
      jsonBuffer["pwSec"] = pwSec;
      // Convert JSON object into a string
      serializeJson(jsonBuffer, wifiCredentials);

      // encode the data
      int keyIndex = 0;
      Serial.println("Stored settings: " + wifiCredentials);
      for (int index = 0; index < wifiCredentials.length(); index++)
      {
        wifiCredentials[index] = (char)wifiCredentials[index] ^ (char)apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName))
          keyIndex = 0;
      }
      Serial.println("Stored encrypted: " + wifiCredentials);

      delay(2000);
      SerialBT.print(wifiCredentials);
      SerialBT.flush();
    }
    else if (jsonBuffer.containsKey("getHostname"))
    {
      Serial.println("BTSerial getHostname request");
      String hostName;
      /** Json object for outgoing data */
      jsonBuffer.clear();
      jsonBuffer["hostname"] = WiFi.getHostname();
      serializeJson(jsonBuffer, hostName);

      // encode the data
      int keyIndex = 0;
      Serial.println("Hostname: " + hostName);
      for (int index = 0; index < hostName.length(); index++)
      {
        hostName[index] = (char)hostName[index] ^ (char)apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName))
          keyIndex = 0;
      }
      Serial.println("Stored encrypted: " + hostName);
      delay(2000);
      SerialBT.print(hostName);
      SerialBT.flush();
    }
    else if (jsonBuffer.containsKey("reset"))
    {
      WiFi.disconnect();
      esp_restart();
    }
  }
  else
  {
    Serial.println("Received invalid JSON");
  }
  jsonBuffer.clear();
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event)
{
  isConnected = true;
  connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event)
{
  isConnected = false;
  connStatusChanged = true;
}

/** Callback for connection loss */
void gotCon(system_event_id_t event)
{
  Serial.println("Connection established, waiting for IP");
}

/** Callback for Station mode start */
void staStart(system_event_id_t event)
{
  Serial.println("Station mode start");
}

/** Callback for Station mode stop */
void staStop(system_event_id_t event)
{
  Serial.println("Station mode stop");
}

/**
   Start connection to AP
*/
void connectWiFi()
{
  SerialBT.end();
  // Setup callback function for successful connection
  WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
  // Setup callback function for lost connection
  WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);
  // Setup callback function for connection established
  WiFi.onEvent(gotCon, SYSTEM_EVENT_STA_CONNECTED);
  // Setup callback function for connection established
  WiFi.onEvent(staStart, SYSTEM_EVENT_STA_START);
  // Setup callback function for connection established
  WiFi.onEvent(staStop, SYSTEM_EVENT_STA_STOP);

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("Start connection to ");
  Serial.print(ssidPrim);
  WiFi.setHostname(apName);
  WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
  WiFi.setHostname(apName);
}

void setup()
{
  // Create unique device name
  createName();
  // Initialize Serial port
  Serial.begin(115200);
  // Send some device info
  Serial.println("Build: ");
  Serial.print(compileDate);
  pinMode(relay_io1, OUTPUT);
  pinMode(relay_io2, OUTPUT);
  pinMode(relay_io3, OUTPUT);
  pinMode(relay_io4, OUTPUT);

  Preferences preferences;
  preferences.begin("WiFiCred", false);
  bool hasPref = preferences.getBool("valid", false);
  if (hasPref)
  {
    ssidPrim = preferences.getString("ssidPrim", "");
    ssidSec = preferences.getString("ssidSec", "");
    pwPrim = preferences.getString("pwPrim", "");
    pwSec = preferences.getString("pwSec", "");

    if (ssidPrim.equals("") || pwPrim.equals("") || ssidSec.equals("") || pwPrim.equals(""))
    {
      Serial.println("Found preferences but credentials are invalid");
    }
    else
    {
      Serial.println("Read from preferences:");
      Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
      Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
      hasCredentials = true;
    }
  }
  else
  {
    Serial.println("Could not find preferences, need send data over BLE");
    initBTSerial();
  }
  preferences.end();
  pinMode(ONBOARD_LED, OUTPUT);
  server.on("/on", [](AsyncWebServerRequest *request) {
    if (request->hasParam(SOCKET))
    {
      String socket = request->getParam(SOCKET)->value();
      if (socket == "1")
      {
        relay1 = true;
      }
      else if (socket == "2")
      {
        relay2 = true;
      }
      else if (socket == "3")
      {
        relay3 = true;
      }
      else if (socket == "4")
      {
        relay4 = true;
      }
    }
    request->send(200, "text/plain", "Ok");
  });

  server.on("/off", [](AsyncWebServerRequest *request) {
    if (request->hasParam(SOCKET))
    {
      String socket = request->getParam(SOCKET)->value();
      if (socket == "1")
      {
        relay1 = false;
      }
      else if (socket == "2")
      {
        relay2 = false;
      }
      else if (socket == "3")
      {
        relay3 = false;
      }
      else if (socket == "4")
      {
        relay4 = false;
      }
    }
    request->send(200, "text/plain", "Ok");
  });

  server.on("/state", [](AsyncWebServerRequest *request) {
    if (request->hasParam(SOCKET))
    {
      String socket = request->getParam(SOCKET)->value();
      if (socket == "1")
      {
        if (relay1)
        {
          request->send(200, "text/plain", "On");
        }
        else
        {
          request->send(200, "text/plain", "Off");
        }
      }
      else if (socket == "2")
      {
        if (relay2)
        {
          request->send(200, "text/plain", "On");
        }
        else
        {
          request->send(200, "text/plain", "Off");
        }
      }
      else if (socket == "3")
      {
        if (relay3)
        {
          request->send(200, "text/plain", "On");
        }
        else
        {
          request->send(200, "text/plain", "Off");
        }
      }
      else if (socket == "4")
      {
        if (relay4)
        {
          request->send(200, "text/plain", "On");
        }
        else
        {
          request->send(200, "text/plain", "Off");
        }
      }
    }
    else
    {
      request->send(200, "text/plain", "No socket specified");
    }
  });

  server.on("/erase", [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Erasing");
    Serial.println("Received erase command");
    Preferences preferences;
    preferences.begin("WiFiCred", false);
    preferences.clear();
    preferences.end();
    connStatusChanged = true;
    hasCredentials = false;
    ssidPrim = "";
    pwPrim = "";
    ssidSec = "";
    pwSec = "";
    int err;
    err = nvs_flash_init();
    Serial.println("nvs_flash_init: " + err);
    err = nvs_flash_erase();
    Serial.println("nvs_flash_erase: " + err);
    esp_restart();
  });
  if (hasCredentials)
  {
    connectWiFi();
  }
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED && btSerialOn != true)
  {
    initBTSerial();
  }
  else if (WiFi.status() == WL_CONNECTED && btSerialOn)
  {
    SerialBT.end();
  }
  if (touchRead(32) <= 1)
  {
    if (relay1)
    {
      relay1 = false;
      delay(500);
    }
    else
    {
      relay1 = true;
      delay(500);
    }
  }
  if (relay1)
  {
    digitalWrite(ONBOARD_LED, HIGH);
    digitalWrite(relay_io1, LOW);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
    digitalWrite(relay_io1, HIGH);
  }
    if (relay2)
  {
    digitalWrite(ONBOARD_LED, HIGH);
    digitalWrite(relay_io2, LOW);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
    digitalWrite(relay_io2, HIGH);
  }
    if (relay3)
  {
    digitalWrite(ONBOARD_LED, HIGH);
    digitalWrite(relay_io3, LOW);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
    digitalWrite(relay_io3, HIGH);
  }
    if (relay4)
  {
    digitalWrite(ONBOARD_LED, HIGH);
    digitalWrite(relay_io4, LOW);
  }
  else
  {
    digitalWrite(ONBOARD_LED, LOW);
    digitalWrite(relay_io4, HIGH);
  }

  if (connStatusChanged)
  {
    if (isConnected)
    {
      server.begin();
      if (MDNS.begin("ljus-240ac46177c4"))
      {
        Serial.println("MDNS responder started");
      }
      Serial.println("Connected to AP: ");
      Serial.print(WiFi.SSID());
      Serial.print(" with IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(" RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println("Hostname: ");
      Serial.print(WiFi.getHostname());
    }
    else
    {
      if (hasCredentials)
      {
        Serial.println("Lost WiFi connection");
        connectWiFi();
      }
    }
    connStatusChanged = false;
  }
  // Check if Data over SerialBT has arrived
  if (SerialBT.available() != 0)
  {
    // Get and parse received data
    readBTSerial();
  }
}
