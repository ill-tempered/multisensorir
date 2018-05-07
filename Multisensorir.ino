/*
Based on the Bruh Automation sensor hardware and original code. Modified, heavily :), by Shawn Corey.

Original code is available at: https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor


To use this code you will need the following dependancies:

- Support for the ESP8266 boards.
- You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json into the Additional Board Managers URL field.
- Next, download the ESP8266 dependancies by going to Tools -> Board -> Board Manager and searching for ESP8266 and installing it.

- You will also need to download the follow libraries by going to Sketch -> Include Libraries -> Manage Libraries
- DHT sensor library
- Adfruit unified sensor
- PubSubClient
- ArduinoJSON

*/

// The packet for the LED discovery registration packet is really big so need to make the max packet size bigger.
// If it still fails then edit the PubSubClient.h file directly,
// had problems without changing the .h file so look there if it fails to register the LED
#define MQTT_MAX_PACKET_SIZE 768

#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>


#include <algorithm>
#include <string>

/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define wifi_ssid "YOURWIFISSID"
#define wifi_password "YOURWIFIPASSWORD"
#define mqtt_server "10.0.0.0"
#define mqtt_user "YOURMQTTUSER"
#define mqtt_password "YOURMQTTPASSWORD"
#define mqtt_port 1883


#define DEVICE_NAME "YOURPREFERREDSENSORNAME"
#define DEVICE_FRIENDLY_NAME "EASYNAME"

#define IR_LED 12 // GPIO the IR LED is connected to/controlled by. GPIO 4 = D2.
#define HTTP_PORT 80 // The port the HTTP server is listening on.
#define HOSTNAME "ir_server" // Name of the device you want in mDNS.


// HTML arguments we will parse for IR code information.
#define argType "type"
#define argData "code"
#define argBits "bits"
#define argRepeat "repeats"
#define DEBUG True

// Sensor name is used for OTA and WiFi hostnames, can be different from DEVICE_NAME if you want
// to have a difference between your MQTT topic names and the OTA/WiFi hostnames
#define SENSORNAME DEVICE_NAME

#define OTApassword "OTAPASS"
int OTAport = 8266;
// Should not need to set anything below this, unless you have changed the MQTT path for HA discovery


//
// MQTT TOPICS (to maintain HomeAssitant discovery ability do not edit unless you know what you are doing)
//

// Topics for general device management, this if for actions like forcing re/un-registration w/ discovery, rebooting, etc
#define DEVICE_DEVICE_COMMAND_TOPIC "homeassistant/" DEVICE_NAME "/set"

// Topics for PIR (motion sensor)
#define DEVICE_PIR_DISCOVERY_TOPIC "homeassistant/binary_sensor/" DEVICE_NAME "_sensor_pir/config"
#define DEVICE_PIR_STATE_TOPIC "homeassistant/binary_sensor/" DEVICE_NAME "_sensor_pir/state"
#define DEVICE_PIR_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " Motion Sensor\",\"device_class\":\"motion\"}"

// Topics for Button
#define DEVICE_BUTTON_DISCOVERY_TOPIC "homeassistant/binary_sensor/" DEVICE_NAME "_button/config"
#define DEVICE_BUTTON_STATE_TOPIC "homeassistant/binary_sensor/" DEVICE_NAME "_button/state"
#define DEVICE_BUTTON_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " Button\"}"

// Topics for temperature sensor
#define DEVICE_TEMP_DISCOVERY_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_temp/config"
#define DEVICE_TEMP_STATE_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_temp/state"
#define DEVICE_TEMP_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Temp\",\"unit_of_measurement\":\"°F\"}"


// Topics for humidity sensor
#define DEVICE_HUMIDITY_DISCOVERY_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_humidity/config"
#define DEVICE_HUMIDITY_STATE_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_humidity/state"
#define DEVICE_HUMIDITY_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Humidity\",\"unit_of_measurement\":\"%\"}"

// Topics for LDR (light sensor)
#define DEVICE_LDR_DISCOVERY_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_ldr/config"
#define DEVICE_LDR_STATE_TOPIC "homeassistant/sensor/" DEVICE_NAME "_sensor_ldr/state"
#define DEVICE_LDR_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Light Level\",\"unit_of_measurement\":\"lux\"}"


// Topics for LED. there are 3 sets because they cover on/off, brightness and RGB color seperately
#define DEVICE_LED_DISCOVERY_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/config"
#define DEVICE_LED_COMMAND_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/set"
#define DEVICE_LED_STATE_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/state"

#define DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/brighness/set"
#define DEVICE_LED_BRIGHTNESS_STATE_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/brighness/state"

#define DEVICE_LED_RGB_COMMAND_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/rgb/set"
#define DEVICE_LED_RGB_STATE_TOPIC "homeassistant/light/" DEVICE_NAME "_sensor_led/rgb/state"

#define DEVICE_LED_DISCOVERY_REGISTER_MESSAGE "{\"name\":\"" DEVICE_FRIENDLY_NAME " LED\",\"brightness\":true,\"flash\":true,\"rgb\":true,\"optomistic\":false,\"qos\":0,"\
"\"command_topic\":\"homeassistant/light/" DEVICE_NAME "_sensor_led/set\","\
"\"brightness_command_topic\":\"homeassistant/light/" DEVICE_NAME "_sensor_led/brighness/set\","\
"\"brightness_state_topic\":\"homeassistant/light/" DEVICE_NAME "_sensor_led/brighness/state\","\
"\"rgb_command_topic\":\"homeassistant/light/" DEVICE_NAME "_sensor_led/rgb/set\","\
"\"rgb_state_topic\":\"homeassistant/light/" DEVICE_NAME "_sensor_led/rgb/state\"}"

// Message text for LED state
#define MQTT_ON_CMD "ON" // command that sets relay on
#define MQTT_OFF_CMD "OFF" // command that sets relay off

// Message text for device commands
#define MQTT_RESET_CMD "reset" // command that resets the device
#define MQTT_STAT_CMD "stat" // command to resend all state
#define MQTT_REGISTER_CMD "register" // command to force reregistration
#define MQTT_UNREGISTER_CMD "unregister" // command to force unregistration

const int redPin = D1;
const int greenPin = D2;
const int bluePin = D3;
const int BTNPIN = D8;
#define PIRPIN D5
#define DHTPIN D7
#define DHTTYPE DHT22
#define LDRPIN A0

// Variables for LDR(light sensor)
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

// Variables for temp sensor
float diffTEMP = 0.2;
float tempValue;

// Variables for humidity sensor
float diffHUM = 1;
float humValue;

// Variables for Button
int BTNValue;
int BTNOldValue;
long BTNTimer;
bool BTNStatus = false;

// Variables for PIR(motion sensor)
int pirValue;
int pirOldValue;
long pirTimer;
bool motionStatus = false;

// Buffers for MQTT messages
char message_buff[100];
char str_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

// Variables for LED
byte red = 0;
byte green = 0;
byte blue = 0;
byte brightness = 0;
// Variables that hold normalized variables that include color and brightness
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = false;

// Globals
ESP8266WebServer server(HTTP_PORT);
IRsend irsend = IRsend(IR_LED);
MDNSResponder mdns;
WiFiManager wifiManager;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

uint16_t *codeArray;
uint32_t lastReconnectAttempt = 0; // MQTT last attempt reconnection number
bool boot = true;
bool ir_lock = false; // Primitive locking for gating the IR LED.
uint32_t sendReqCounter = 0;

// Debug messages get sent to the serial port.
void debug(String str) {
#ifdef DEBUG
uint32_t now = millis();
Serial.printf("%07u.%03u: %s\n", now / 1000, now % 1000, str.c_str());
#endif // DEBUG
}

uint64_t getUInt64fromHex(char const *str) {
uint64_t result = 0;
uint16_t offset = 0;
// Skip any leading '0x' or '0X' prefix.
if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
offset = 2;
for (; isxdigit((unsigned char)str[offset]); offset++) {
char c = str[offset];
result *= 16;
if (isdigit(c)) /* '0' .. '9' */
result += c - '0';
else if (isupper(c)) /* 'A' .. 'F' */
result += c - 'A' + 10;
else /* 'a' .. 'f'*/
result += c - 'a' + 10;
}
return result;
}

String timeSince(uint32_t const start) {
if (start == 0)
return "Never";
uint32_t diff = 0;
uint32_t now = millis();
if (start < now)
diff = now - start;
else
diff = UINT32_MAX - start + now;
diff /= 1000; // Convert to seconds.
if (diff == 0) return "Now";

// Note: millis() can only count up to 45 days, so uint8_t is safe.
uint8_t days = diff / (60 * 60 * 24);
uint8_t hours = (diff / (60 * 60)) % 24;
uint8_t minutes = (diff / 60) % 60;
uint8_t seconds = diff % 60;

String result = "";
if (days)
result += String(days) + " day";
if (days > 1) result += "s";
if (hours)
result += " " + String(hours) + " hour";
if (hours > 1) result += "s";
if (minutes)
result += " " + String(minutes) + " minute";
if (minutes > 1) result += "s";
if (seconds)
result += " " + String(seconds) + " second";
if (seconds > 1) result += "s";
result.trim();
return result + " ago";
}

// Reset web page
void handleReset() {
server.send(200, "text/html",
"<html><head><title>Reset Config</title></head>"
"<body>"
"<h1>Resetting the WiFiManager config back to defaults.</h1>"
"<p>Device restarting. Try connecting in a few seconds.</p>"
"</body></html>");
// Do the reset.
wifiManager.resetSettings();
delay(10);
ESP.restart();
delay(1000);
}

// Quick and dirty check for any unsafe chars in a string
// that may cause HTML shenanigans. e.g. An XSS.
bool hasUnsafeHTMLChars(String input) {
static char unsafe[] = "';!-\"<>=&{}()";
for (uint8_t i = 0; unsafe[i]; i++)
if (input.indexOf(unsafe[i]) != -1) return true;
return false;
}

// Root web page with example usage etc.
void handleRoot() {
server.send(200, "text/html",
"<html><head><title>IR Web server</title></head>"
"<body>"
"<center><h1>ESP8266 IR Web Server</h1></center>"
"<br><hr>"
"<h3>Information</h3>"
"<p>IP address: " + WiFi.localIP().toString() + "<br>"
"Booted: " + timeSince(1) + "<br>" +
"IR Lib Version: " _IRREMOTEESP8266_VERSION_ "<br>"
"Total send requests: " + String(sendReqCounter) + "</p>"
"<br><hr>"
"<h3>Hardcoded examples</h3>"
"<p><a href=\"ir?code=38000,1,69,341,171,21,64,21,64,21,21,21,21,21,21,21,"
"21,21,21,21,64,21,64,21,21,21,64,21,21,21,21,21,21,21,64,21,21,21,64,"
"21,21,21,21,21,21,21,64,21,21,21,21,21,21,21,21,21,64,21,64,21,64,21,"
"21,21,64,21,64,21,64,21,1600,341,85,21,3647&type=31\">"
"Sherwood Amp On (GlobalCache)</a></p>"
"<p><a href=\"ir?code=38000,8840,4446,546,1664,546,1664,546,546,546,546,"
"546,546,546,546,546,546,546,1664,546,1664,546,546,546,1664,546,546,"
"546,546,546,546,546,1664,546,546,546,1664,546,546,546,1664,546,1664,"
"546,1664,546,546,546,546,546,546,546,546,546,1664,546,546,546,546,546,"
"546,546,1664,546,1664,546,1664,546,41600,8840,2210,546&type=30\">"
"Sherwood Amp Off (Raw)</a></p>"
"<p><a href=\"ir?code=0000,006E,0022,0002,0155,00AA,0015,0040,0015,0040"
",0015,0015,0015,0015,0015,0015,0015,0015,0015,0015,0015,0040,0015,0040"
",0015,0015,0015,0040,0015,0015,0015,0015,0015,0015,0015,0040,0015,0015"
",0015,0015,0015,0040,0015,0040,0015,0015,0015,0015,0015,0015,0015,0015"
",0015,0015,0015,0040,0015,0015,0015,0015,0015,0040,0015,0040,0015,0040"
",0015,0040,0015,0040,0015,0640,0155,0055,0015,0E40"
"&type=25&repeats=1\">"
"Sherwood Amp Input TAPE (Pronto)</a></p>"
"<p><a href=\"ir?type=7&code=E0E09966\">TV on (Samsung)</a></p>"
"<p><a href=\"ir?type=4&code=0xf50&bits=12\">Power Off (Sony 12bit)</a></p>"
"<br><hr>"
"<h3>Send a simple IR message</h3><p>"
"<form method='POST' action='/ir' enctype='multipart/form-data'>"
"Type: "
"<select name='type'>"
"<option value='9'>Aiwa RC T501</option>"
"<option value='37'>Carrier AC</option>"
"<option value='15'>Coolix</option>"
"<option value='17'>Denon</option>"
"<option value='13'>Dish</option>"
"<option value='6'>JVC</option>"
"<option value='10'>LG</option>"
"<option value='36'>Lasertag</option>"
"<option value='35'>MagiQuest</option>"
"<option value='34'>Midea</option>"
"<option value='12'>Mitsubishi</option>"
"<option value='39'>Mitsubishi2</option>"
"<option selected='selected' value='3'>NEC</option>" // Default
"<option value='29'>Nikai</option>"
"<option value='5'>Panasonic</option>"
"<option value='1'>RC-5</option>"
"<option value='23'>RC-5X</option>"
"<option value='2'>RC-6</option>"
"<option value='21'>RC-MM</option>"
"<option value='7'>Samsung</option>"
"<option value='11'>Sanyo</option>"
"<option value='22'>Sanyo LC7461</option>"
"<option value='14'>Sharp</option>"
"<option value='19'>Sherwood</option>"
"<option value='4'>Sony</option>"
"<option value='8'>Whynter</option>"
"</select>"
" Code: 0x<input type='text' name='code' min='0' value='0' size='16'"
" maxlength='16'>"
" Bit size: "
"<select name='bits'>"
"<option selected='selected' value='0'>Default</option>" // Default
// Common bit length options for most protocols.
"<option value='12'>12</option>"
"<option value='13'>13</option>"
"<option value='14'>14</option>"
"<option value='15'>15</option>"
"<option value='16'>16</option>"
"<option value='20'>20</option>"
"<option value='21'>21</option>"
"<option value='24'>24</option>"
"<option value='28'>28</option>"
"<option value='32'>32</option>"
"<option value='36'>36</option>"
"<option value='48'>48</option>"
"<option value='56'>56</option>"
"</select>"
" Repeats: <input type='number' name='repeats' min='0' max='99' value='0'"
"size='2' maxlength='2'>"
" <input type='submit' value='Send IR'>"
"</form>"
"<br><hr>"
"<h3>Send an IRremote Raw IR message</h3><p>"
"<form method='POST' action='/ir' enctype='multipart/form-data'>"
"<input type='hidden' name='type' value='30'>"
"String: (freq,array data) <input type='text' name='code' size='132'"
" value='38000,4420,4420,520,1638,520,1638,520,1638,520,520,520,520,520,"
"520,520,520,520,520,520,1638,520,1638,520,1638,520,520,520,"
"520,520,520,520,520,520,520,520,520,520,1638,520,520,520,520,520,"
"520,520,520,520,520,520,520,520,1638,520,520,520,1638,520,1638,520,"
"1638,520,1638,520,1638,520,1638,520'>"
" <input type='submit' value='Send Raw'>"
"</form>"
"<br><hr>"
"<h3>Send a <a href='https://irdb.globalcache.com/'>GlobalCache</a>"
" IR message</h3><p>"
"<form method='POST' action='/ir' enctype='multipart/form-data'>"
"<input type='hidden' name='type' value='31'>"
"String: 1:1,1,<input type='text' name='code' size='132'"
" value='38000,1,1,170,170,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,"
"20,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,20,20,20,20,63,20,"
"20,20,20,20,20,20,20,20,20,20,20,20,63,20,20,20,63,20,63,20,63,20,"
"63,20,63,20,63,20,1798'>"
" <input type='submit' value='Send GlobalCache'>"
"</form>"
"<br><hr>"
"<h3>Send a <a href='http://www.remotecentral.com/cgi-bin/files/rcfiles.cgi"
"?area=pronto&db=discrete'>Pronto code</a> IR message</h3><p>"
"<form method='POST' action='/ir' enctype='multipart/form-data'>"
"<input type='hidden' name='type' value='25'>"
"String (comma separated): <input type='text' name='code' size='132'"
" value='0000,0067,0000,0015,0060,0018,0018,0018,0030,0018,0030,0018,"
"0030,0018,0018,0018,0030,0018,0018,0018,0018,0018,0030,0018,0018,"
"0018,0030,0018,0030,0018,0030,0018,0018,0018,0018,0018,0030,0018,"
"0018,0018,0018,0018,0030,0018,0018,03f6'>"
" Repeats: <input type='number' name='repeats' min='0' max='99' value='0'"
"size='2' maxlength='2'>"
" <input type='submit' value='Send Pronto'>"
"</form>"
"<br><hr>"
"<h3>Send an Air Conditioner IR message</h3><p>"
"<form method='POST' action='/ir' enctype='multipart/form-data'>"
"Type: "
"<select name='type'>"
"<option value='27'>Argo</option>"
"<option value='16'>Daikin</option>"
"<option value='33'>Fujitsu</option>"
"<option value='24'>Gree</option>"
"<option value='38'>Haier</option>"
"<option selected='selected' value='18'>Kelvinator</option>" // Default
"<option value='20'>Mitsubishi</option>"
"<option value='32'>Toshiba</option>"
"<option value='28'>Trotec</option>"
"</select>"
" State code: 0x"
"<input type='text' name='code' size='" + String(STATE_SIZE_MAX * 2) +
"' maxlength='" + String(STATE_SIZE_MAX * 2) + "'"
" value='190B8050000000E0190B8070000010F0'>"
" <input type='submit' value='Send A/C State'>"
"</form>"
"<br><hr>"
"<h3>Update IR Server firmware</h3><p>"
"<b><mark>Warning:</mark></b><br> "
"<i>Updating your firmware may screw up your access to the device. "
"If you are going to use this, know what you are doing first "
"(and you probably do).</i><br>"
"<form method='POST' action='/update' enctype='multipart/form-data'>"
"Firmware to upload: <input type='file' name='update'>"
"<input type='submit' value='Update'>"
"</form>"
"</body></html>");
}


// Parse an Air Conditioner A/C Hex String/code and send it.
// Args:
// irType: Nr. of the protocol we need to send.
// str: A hexadecimal string containing the state to be sent.
void parseStringAndSendAirCon(const uint16_t irType, const String str) {
uint8_t strOffset = 0;
uint8_t state[STATE_SIZE_MAX] = {0}; // All array elements are set to 0.
uint16_t stateSize = 0;

if (str.startsWith("0x") || str.startsWith("0X"))
strOffset = 2;
// Calculate how many hexadecimal characters there are.
uint16_t inputLength = str.length() - strOffset;
if (inputLength == 0) {
debug("Zero length AirCon code encountered. Ignored.");
return; // No input. Abort.
}

switch (irType) { // Get the correct state size for the protocol.
case KELVINATOR:
stateSize = KELVINATOR_STATE_LENGTH;
break;
case TOSHIBA_AC:
stateSize = TOSHIBA_AC_STATE_LENGTH;
break;
case DAIKIN:
stateSize = DAIKIN_COMMAND_LENGTH;
break;
case MITSUBISHI_AC:
stateSize = MITSUBISHI_AC_STATE_LENGTH;
break;
case TROTEC:
stateSize = TROTEC_COMMAND_LENGTH;
break;
case ARGO:
stateSize = ARGO_COMMAND_LENGTH;
break;
case GREE:
stateSize = GREE_STATE_LENGTH;
break;
case FUJITSU_AC:
// Fujitsu has four distinct & different size states, so make a best guess
// which one we are being presented with based on the number of
// hexadecimal digits provided. i.e. Zero-pad if you need to to get
// the correct length/byte size.
stateSize = inputLength / 2; // Every two hex chars is a byte.
// Use at least the minimum size.
stateSize = std::max(stateSize,
(uint16_t) (FUJITSU_AC_STATE_LENGTH_SHORT - 1));
// If we think it isn't a "short" message.
if (stateSize > FUJITSU_AC_STATE_LENGTH_SHORT)
// Then it has to be at least the smaller version of the "normal" size.
stateSize = std::max(stateSize,
(uint16_t) (FUJITSU_AC_STATE_LENGTH - 1));
// Lastly, it should never exceed the maximum "normal" size.
stateSize = std::min(stateSize, (uint16_t) FUJITSU_AC_STATE_LENGTH);
break;
case HAIER_AC:
stateSize = HAIER_AC_STATE_LENGTH;
break;
case HITACHI_AC:
stateSize = HITACHI_AC_STATE_LENGTH;
break;
default: // Not a protocol we expected. Abort.
debug("Unexpected AirCon protocol detected. Ignoring.");
return;
}
if (inputLength > stateSize * 2) {
debug("AirCon code to large for the given protocol.");
return;
}

// Ptr to the least significant byte of the resulting state for this protocol.
uint8_t *statePtr = &state[stateSize - 1];

// Convert the string into a state array of the correct length.
for (uint16_t i = 0; i < inputLength; i++) {
// Grab the next least sigificant hexadecimal digit from the string.
uint8_t c = tolower(str[inputLength + strOffset - i - 1]);
if (isxdigit(c)) {
if (isdigit(c))
c -= '0';
else
c = c - 'a' + 10;
} else {
debug("Aborting! Non-hexadecimal char found in AirCon state: " + str);
return;
}
if (i % 2 == 1) { // Odd: Upper half of the byte.
*statePtr += (c << 4);
statePtr--; // Advance up to the next least significant byte of state.
} else { // Even: Lower half of the byte.
*statePtr = c;
}
}

// Make the appropriate call for the protocol type.
switch (irType) {
#if SEND_KELVINATOR
case KELVINATOR:
irsend.sendKelvinator(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_TOSHIBA_AC
case TOSHIBA_AC:
irsend.sendToshibaAC(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_DAIKIN
case DAIKIN:
irsend.sendDaikin(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if MITSUBISHI_AC
case MITSUBISHI_AC:
irsend.sendMitsubishiAC(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_TROTEC
case TROTEC:
irsend.sendTrotec(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_ARGO
case ARGO:
irsend.sendArgo(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_GREE
case GREE:
irsend.sendGree(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_FUJITSU_AC
case FUJITSU_AC:
irsend.sendFujitsuAC(reinterpret_cast<uint8_t *>(state), stateSize);
break;
#endif
#if SEND_HAIER_AC
case HAIER_AC:
irsend.sendHaierAC(reinterpret_cast<uint8_t *>(state));
break;
#endif
#if SEND_HITACHI_AC
case HITACHI_AC:
irsend.sendHitachiAC(reinterpret_cast<uint8_t *>(state));
break;
#endif
}
}

// Count how many values are in the String.
// Args:
// str: String containing the values.
// sep: Character that separates the values.
// Returns:
// The number of values found in the String.
uint16_t countValuesInStr(const String str, char sep) {
int16_t index = -1;
uint16_t count = 1;
do {
index = str.indexOf(sep, index + 1);
count++;
} while (index != -1);
return count;
}

// Dynamically allocate an array of uint16_t's.
// Args:
// size: Nr. of uint16_t's need to be in the new array.
// Returns:
// A Ptr to the new array. Restarts the ESP8266 if it fails.
uint16_t * newCodeArray(const uint16_t size) {
uint16_t *result;

result = reinterpret_cast<uint16_t*>(malloc(size * sizeof(uint16_t)));
// Check we malloc'ed successfully.
if (result == NULL) { // malloc failed, so give up.
Serial.printf("\nCan't allocate %d bytes. (%d bytes free)\n",
size * sizeof(uint16_t), ESP.getFreeHeap());
Serial.println("Giving up & forcing a reboot.");
ESP.restart(); // Reboot.
delay(500); // Wait for the restart to happen.
return result; // Should never get here, but just in case.
}
return result;
}

#if SEND_GLOBALCACHE
// Parse a GlobalCache String/code and send it.
// Args:
// str: A GlobalCache formatted String of comma separated numbers.
// e.g. "38000,1,1,170,170,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,
// 20,20,63,20,63,20,63,20,20,20,20,20,20,20,20,20,20,20,20,20,63,
// 20,20,20,20,20,20,20,20,20,20,20,20,20,63,20,20,20,63,20,63,20,
// 63,20,63,20,63,20,63,20,1798"
// Note: The leading "1:1,1," of normal GC codes should be removed.
void parseStringAndSendGC(const String str) {
uint16_t count;
uint16_t *code_array;
String tmp_str;

// Remove the leading "1:1,1," if present.
if (str.startsWith("1:1,1,"))
tmp_str = str.substring(6);
else
tmp_str = str;

// Find out how many items there are in the string.
count = countValuesInStr(tmp_str, ',');

// Now we know how many there are, allocate the memory to store them all.
code_array = newCodeArray(count);

// Now convert the strings to integers and place them in code_array.
count = 0;
uint16_t start_from = 0;
int16_t index = -1;
do {
index = tmp_str.indexOf(',', start_from);
code_array[count] = tmp_str.substring(start_from, index).toInt();
start_from = index + 1;
count++;
} while (index != -1);

irsend.sendGC(code_array, count); // All done. Send it.
free(code_array); // Free up the memory allocated.
}
#endif // SEND_GLOBALCACHE

#if SEND_PRONTO
// Parse a Pronto Hex String/code and send it.
// Args:
// str: A comma-separated String of nr. of repeats, then hexadecimal numbers.
// e.g. "R1,0000,0067,0000,0015,0060,0018,0018,0018,0030,0018,0030,0018,
// 0030,0018,0018,0018,0030,0018,0018,0018,0018,0018,0030,0018,
// 0018,0018,0030,0018,0030,0018,0030,0018,0018,0018,0018,0018,
// 0030,0018,0018,0018,0018,0018,0030,0018,0018,03f6"
// or
// "0000,0067,0000,0015,0060,0018". i.e. without the Repeat value
// Requires at least PRONTO_MIN_LENGTH comma-separated values.
// sendPronto() only supports raw pronto code types, thus so does this.
// repeats: Nr. of times the message is to be repeated.
// This value is ignored if an embeddd repeat is found in str.
void parseStringAndSendPronto(const String str, uint16_t repeats) {
uint16_t count;
uint16_t *code_array;
int16_t index = -1;
uint16_t start_from = 0;

// Find out how many items there are in the string.
count = countValuesInStr(str, ',');

// Check if we have the optional embedded repeats value in the code string.
if (str.startsWith("R") || str.startsWith("r")) {
// Grab the first value from the string, as it is the nr. of repeats.
index = str.indexOf(',', start_from);
repeats = str.substring(start_from + 1, index).toInt(); // Skip the 'R'.
start_from = index + 1;
count--; // We don't count the repeats value as part of the code array.
}

// We need at least PRONTO_MIN_LENGTH values for the code part.
if (count < PRONTO_MIN_LENGTH) return;

// Now we know how many there are, allocate the memory to store them all.
code_array = newCodeArray(count);

// Rest of the string are values for the code array.
// Now convert the hex strings to integers and place them in code_array.
count = 0;
do {
index = str.indexOf(',', start_from);
// Convert the hexadecimal value string to an unsigned integer.
code_array[count] = strtoul(str.substring(start_from, index).c_str(),
NULL, 16);
start_from = index + 1;
count++;
} while (index != -1);

irsend.sendPronto(code_array, count, repeats); // All done. Send it.
free(code_array); // Free up the memory allocated.
}
#endif // SEND_PRONTO

#if SEND_RAW
// Parse an IRremote Raw Hex String/code and send it.
// Args:
// str: A comma-separated String containing the freq and raw IR data.
// e.g. "38000,9000,4500,600,1450,600,900,650,1500,..."
// Requires at least two comma-separated values.
// First value is the transmission frequency in Hz or kHz.
void parseStringAndSendRaw(const String str) {
uint16_t count;
uint16_t freq = 38000; // Default to 38kHz.
uint16_t *raw_array;

// Find out how many items there are in the string.
count = countValuesInStr(str, ',');

// We expect the frequency as the first comma separated value, so we need at
// least two values. If not, bail out.
if (count < 2) return;
count--; // We don't count the frequency value as part of the raw array.

// Now we know how many there are, allocate the memory to store them all.
raw_array = newCodeArray(count);

// Grab the first value from the string, as it is the frequency.
int16_t index = str.indexOf(',', 0);
freq = str.substring(0, index).toInt();
uint16_t start_from = index + 1;
// Rest of the string are values for the raw array.
// Now convert the strings to integers and place them in raw_array.
count = 0;
do {
index = str.indexOf(',', start_from);
raw_array[count] = str.substring(start_from, index).toInt();
start_from = index + 1;
count++;
} while (index != -1);

irsend.sendRaw(raw_array, count, freq); // All done. Send it.
free(raw_array); // Free up the memory allocated.
}
#endif // SEND_RAW

// Parse the URL args to find the IR code.
void handleIr() {
uint64_t data = 0;
String data_str = "";
int ir_type = 3; // Default to NEC codes.
uint16_t nbits = 0;
uint16_t repeat = 0;

for (uint16_t i = 0; i < server.args(); i++) {
if (server.argName(i) == argType)
ir_type = atoi(server.arg(i).c_str());
if (server.argName(i) == argData) {
data = getUInt64fromHex(server.arg(i).c_str());
data_str = server.arg(i);
}
if (server.argName(i) == argBits)
nbits = atoi(server.arg(i).c_str());
if (server.argName(i) == argRepeat)
repeat = atoi(server.arg(i).c_str());
}
debug("New code received via HTTP");
sendIRCode(ir_type, data, data_str.c_str(), nbits, repeat);
handleRoot();
}

void handleNotFound() {
String message = "File Not Found\n\n";
message += "URI: ";
message += server.uri();
message += "\nMethod: ";
message += (server.method() == HTTP_GET)?"GET":"POST";
message += "\nArguments: ";
message += server.args();
message += "\n";
for (uint8_t i=0; i < server.args(); i++)
message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
server.send(404, "text/plain", message);
}

void setup() {

irsend.begin();
Serial.begin(115200);

pinMode(PIRPIN, INPUT);
pinMode(DHTPIN, INPUT);
pinMode(LDRPIN, INPUT);
pinMode(BTNPIN, INPUT);

//new code to turn LED on then off
analogWrite(redPin, 1);
analogWrite(greenPin, 1);
analogWrite(bluePin, 1);
analogWrite(redPin, 0);
analogWrite(greenPin, 0);
analogWrite(bluePin, 0);

Serial.begin(115200);
delay(10);

Serial.print("calibrating sensor ");
for (int i = 0; i < calibrationTime; i++) {
Serial.print(".");
delay(1000);
}

Serial.println("Starting Node named " SENSORNAME);

setup_wifi();
if (mdns.begin(HOSTNAME, WiFi.localIP())) {
debug("MDNS responder started");
}

// Setup the root web page.
server.on("/", handleRoot);
// Setup the page to handle web-based IR codes.
server.on("/ir", handleIr);
// Setup a reset page to cause WiFiManager information to be reset.
server.on("/reset", handleReset);

// Setup the URL to allow Over-The-Air (OTA) firmware updates.
server.on("/update", HTTP_POST, [](){
server.sendHeader("Connection", "close");
server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
ESP.restart();
}, [](){
HTTPUpload& upload = server.upload();
if (upload.status == UPLOAD_FILE_START) {
WiFiUDP::stopAll();
debug("Update: " + upload.filename);
uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) &
0xFFFFF000;
if (!Update.begin(maxSketchSpace)) { // start with max available size
#ifdef DEBUG
Update.printError(Serial);
#endif // DEBUG
}
} else if (upload.status == UPLOAD_FILE_WRITE) {
if (Update.write(upload.buf, upload.currentSize) !=
upload.currentSize) {
#ifdef DEBUG
Update.printError(Serial);
#endif // DEBUG
}
} else if (upload.status == UPLOAD_FILE_END) {
if (Update.end(true)) { // true to set the size to the current progress
debug("Update Success: " + (String) upload.totalSize +
"\nRebooting...");
}
}
yield();
});

// Set up an error page.
server.onNotFound(handleNotFound);

server.begin();
debug("HTTP server started");

client.setServer(mqtt_server, mqtt_port);
client.setCallback(callback);

ArduinoOTA.setPort(OTAport);
ArduinoOTA.setHostname(HOSTNAME);
ArduinoOTA.setPassword((const char *)OTApassword);

ArduinoOTA.onStart([]() {
Serial.println("Starting");
});
ArduinoOTA.onEnd([]() {
Serial.println("\nEnd");
});
ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
});
ArduinoOTA.onError([](ota_error_t error) {
Serial.printf("Error[%u]: ", error);
switch(error){
case OTA_AUTH_ERROR: Serial.println("Auth Failed"); break;
case OTA_BEGIN_ERROR: Serial.println("Begin Failed"); break;
case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
case OTA_END_ERROR: Serial.println("End Failed"); break;
default: Serial.println("Unknown Error"); break;
}});
ArduinoOTA.begin();
Serial.println("Ready");
configColor(red, green, blue);
}


void setup_wifi() {
delay(10);
Serial.println();
Serial.print("Connecting to ");
Serial.println(wifi_ssid);

WiFi.mode(WIFI_STA);
WiFi.hostname(HOSTNAME);
WiFi.begin(wifi_ssid, wifi_password);

while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
}

Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());

if (mdns.begin(HOSTNAME, WiFi.localIP())) {
debug("MDNS responder started");
}
}


bool sendState(char* topic, const char* message, bool retain = true) {
return client.publish(topic, message, retain);
}


void sendAllState() {
// Send LED status
sprintf(message_buff, "%s", (stateOn) ? MQTT_ON_CMD : MQTT_OFF_CMD);
sendState(DEVICE_LED_STATE_TOPIC, message_buff);

// Send LED RGB values
sprintf(message_buff, "%d,%d,%d", red, green, blue);
sendState(DEVICE_LED_RGB_STATE_TOPIC, message_buff);

// Send LED Brighness value
sprintf(message_buff, "%d", brightness);
sendState(DEVICE_LED_BRIGHTNESS_STATE_TOPIC, message_buff);

// Send Humidity value
dtostrf(humValue, 4, 2, str_buff);
sprintf(message_buff, "%s", str_buff);
sendState(DEVICE_HUMIDITY_STATE_TOPIC, message_buff);

// Send Temperature value
dtostrf(tempValue, 4, 2, str_buff);
sprintf(message_buff, "%s", str_buff);
sendState(DEVICE_TEMP_STATE_TOPIC, message_buff);

// Send Light sensor value
sprintf(message_buff, "%d", LDR);
sendState(DEVICE_LDR_STATE_TOPIC, message_buff);

// Send Motion status
sprintf(message_buff, "%s", (motionStatus) ? MQTT_ON_CMD : MQTT_OFF_CMD);
sendState(DEVICE_PIR_STATE_TOPIC, message_buff);

// Send Button status
sprintf(message_buff, "%s", (BTNStatus) ? MQTT_ON_CMD : MQTT_OFF_CMD);
sendState(DEVICE_BUTTON_STATE_TOPIC, message_buff);
}


void registerSensors(bool forceRegister = false){
sendState(DEVICE_PIR_DISCOVERY_TOPIC, DEVICE_PIR_DISCOVERY_REGISTER_MESSAGE);
sendState(DEVICE_TEMP_DISCOVERY_TOPIC, DEVICE_TEMP_DISCOVERY_REGISTER_MESSAGE);
sendState(DEVICE_HUMIDITY_DISCOVERY_TOPIC, DEVICE_HUMIDITY_DISCOVERY_REGISTER_MESSAGE);
sendState(DEVICE_LDR_DISCOVERY_TOPIC, DEVICE_LDR_DISCOVERY_REGISTER_MESSAGE);
sendState(DEVICE_LED_DISCOVERY_TOPIC, DEVICE_LED_DISCOVERY_REGISTER_MESSAGE);
sendState(DEVICE_BUTTON_DISCOVERY_TOPIC, DEVICE_BUTTON_DISCOVERY_REGISTER_MESSAGE);

}

void unregisterSensors(){
if(!sendState(DEVICE_PIR_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister motion sensor");

if(!sendState(DEVICE_BUTTON_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister Button");

if(!sendState(DEVICE_TEMP_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister temperature sensor");

if(!sendState(DEVICE_HUMIDITY_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister humidity sensor");

if(!sendState(DEVICE_LDR_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister LDR sensor");

if(!sendState(DEVICE_LED_DISCOVERY_TOPIC, ""))
Serial.println("Failed to unregister LED");
}

void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");

char message[length + 1];
for (int i = 0; i < length; i++) {
message[i] = (char)payload[i];
}
message[length] = '\0';
Serial.println(message);

if(strcmp(topic,DEVICE_LED_COMMAND_TOPIC)==0){
if(strcmp(message, MQTT_ON_CMD)==0){
stateOn = true;
configColor(red, green, blue);
} else {
stateOn = false;
configColor(red, green, blue);
}
// Send LED status
sendState(DEVICE_LED_STATE_TOPIC, (stateOn) ? MQTT_ON_CMD : MQTT_OFF_CMD);
} else if(strcmp(topic, DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC)==0){
brightness = atoi(message);
configColor(red, green, blue);
// Send LED Brighness value
sprintf(message_buff, "%d", brightness);
sendState(DEVICE_LED_BRIGHTNESS_STATE_TOPIC, message_buff);
} else if(strcmp(topic, DEVICE_LED_RGB_COMMAND_TOPIC)==0){
char* color = strtok(message, ",");
red = atoi(color);
color = strtok(0, ",");
green = atoi(color);
color = strtok(0, ",");
blue = atoi(color);
configColor(red, green, blue);
// Send LED RGB values
sprintf(message_buff, "%d,%d,%d", red, green, blue);
sendState(DEVICE_LED_RGB_STATE_TOPIC, message_buff);
} else if(strcmp(topic, DEVICE_DEVICE_COMMAND_TOPIC)==0){
if(strcmp(message, MQTT_RESET_CMD)==0){
Serial.println("Restarting device");
ESP.restart();
} else if(strcmp(message, MQTT_STAT_CMD)==0){
Serial.println("Sending all sensor state");
sendAllState();
} else if(strcmp(message, MQTT_REGISTER_CMD)==0){
Serial.println("Forcing registration of sensor");
registerSensors(true);
sendAllState();
} else if(strcmp(message, MQTT_UNREGISTER_CMD)==0){
Serial.println("Forcing unregistration of sensor");
unregisterSensors();
}
}
}


void configColor(int inR, int inG, int inB) {
realRed = map(red, 0, 255, 0, brightness);
realGreen = map(green, 0, 255, 0, brightness);
realBlue = map(blue, 0, 255, 0, brightness);
if(stateOn){
setColor(realRed, realGreen, realBlue);
} else {
setColor(0,0,0);
}
}


// Transmit the given IR message.
//
// Args:
// ir_type: enum of the protocol to be sent.
// code: Numeric payload of the IR message. Most protocols use this.
// code_str: The unparsed code to be sent. Used by complex protocol encodings.
// bits: Nr. of bits in the protocol. 0 means use the protocol's default.
// repeat: Nr. of times the message is to be repeated. (Not all protcols.)
void sendIRCode(int const ir_type, uint64_t const code, char const * code_str,
uint16_t bits, uint16_t repeat) {
// Create a pseudo-lock so we don't try to send two codes at the same time.
while (ir_lock)
delay(20);
ir_lock = true;

// send the IR message.
switch (ir_type) {
#if SEND_RC5
case RC5: // 1
if (bits == 0)
bits = RC5_BITS;
irsend.sendRC5(code, bits, repeat);
break;
#endif
#if SEND_RC6
case RC6: // 2
if (bits == 0)
bits = RC6_MODE0_BITS;
irsend.sendRC6(code, bits, repeat);
break;
#endif
#if SEND_NEC
case NEC: // 3
if (bits == 0)
bits = NEC_BITS;
irsend.sendNEC(code, bits, repeat);
break;
#endif
#if SEND_SONY
case SONY: // 4
if (bits == 0)
bits = SONY_12_BITS;
repeat = std::max(repeat, (uint16_t) SONY_MIN_REPEAT);
irsend.sendSony(code, bits, repeat);
break;
#endif
#if SEND_PANASONIC
case PANASONIC: // 5
if (bits == 0)
bits = PANASONIC_BITS;
irsend.sendPanasonic64(code, bits, repeat);
break;
#endif
#if SEND_JVC
case JVC: // 6
if (bits == 0)
bits = JVC_BITS;
irsend.sendJVC(code, bits, repeat);
break;
#endif
#if SEND_SAMSUNG
case SAMSUNG: // 7
if (bits == 0)
bits = SAMSUNG_BITS;
irsend.sendSAMSUNG(code, bits, repeat);
break;
#endif
#if SEND_WHYNTER
case WHYNTER: // 8
if (bits == 0)
bits = WHYNTER_BITS;
irsend.sendWhynter(code, bits, repeat);
break;
#endif
#if SEND_AIWA_RC_T501
case AIWA_RC_T501: // 9
if (bits == 0)
bits = AIWA_RC_T501_BITS;
repeat = std::max(repeat, (uint16_t) AIWA_RC_T501_MIN_REPEAT);
irsend.sendAiwaRCT501(code, bits, repeat);
break;
#endif
#if SEND_LG
case LG: // 10
if (bits == 0)
bits = LG_BITS;
irsend.sendLG(code, bits, repeat);
break;
#endif
#if SEND_MITSUBISHI
case MITSUBISHI: // 12
if (bits == 0)
bits = MITSUBISHI_BITS;
repeat = std::max(repeat, (uint16_t) MITSUBISHI_MIN_REPEAT);
irsend.sendMitsubishi(code, bits, repeat);
break;
#endif
#if SEND_DISH
case DISH: // 13
if (bits == 0)
bits = DISH_BITS;
repeat = std::max(repeat, (uint16_t) DISH_MIN_REPEAT);
irsend.sendDISH(code, bits, repeat);
break;
#endif
#if SEND_SHARP
case SHARP: // 14
if (bits == 0)
bits = SHARP_BITS;
irsend.sendSharpRaw(code, bits, repeat);
break;
#endif
#if SEND_COOLIX
case COOLIX: // 15
if (bits == 0)
bits = COOLIX_BITS;
irsend.sendCOOLIX(code, bits, repeat);
break;
#endif
case DAIKIN: // 16
case KELVINATOR: // 18
case MITSUBISHI_AC: // 20
case GREE: // 24
case ARGO: // 27
case TROTEC: // 28
case TOSHIBA_AC: // 32
case FUJITSU_AC: // 33
case HAIER_AC: // 38
case HITACHI_AC: // 40
parseStringAndSendAirCon(ir_type, code_str);
break;
#if SEND_DENON
case DENON: // 17
if (bits == 0)
bits = DENON_BITS;
irsend.sendDenon(code, bits, repeat);
break;
#endif
#if SEND_SHERWOOD
case SHERWOOD: // 19
if (bits == 0)
bits = SHERWOOD_BITS;
repeat = std::max(repeat, (uint16_t) SHERWOOD_MIN_REPEAT);
irsend.sendSherwood(code, bits, repeat);
break;
#endif
#if SEND_RCMM
case RCMM: // 21
if (bits == 0)
bits == RCMM_BITS;
irsend.sendRCMM(code, bits, repeat);
break;
#endif
#if SEND_SANYO
case SANYO_LC7461: // 22
if (bits == 0)
bits = SANYO_LC7461_BITS;
irsend.sendSanyoLC7461(code, bits, repeat);
break;
#endif
#if SEND_RC5
case RC5X: // 23
if (bits == 0)
bits = RC5X_BITS;
irsend.sendRC5(code, bits, repeat);
break;
#endif
#if SEND_PRONTO
case PRONTO: // 25
parseStringAndSendPronto(code_str, repeat);
break;
#endif
#if SEND_NIKAI
case NIKAI: // 29
if (bits == 0)
bits = NIKAI_BITS;
irsend.sendNikai(code, bits, repeat);
break;
#endif
#if SEND_RAW
case RAW: // 30
parseStringAndSendRaw(code_str);
break;
#endif
#if SEND_GLOBALCACHE
case GLOBALCACHE: // 31
parseStringAndSendGC(code_str);
break;
#endif
#if SEND_MIDEA
case MIDEA: // 34
if (bits == 0)
bits = MIDEA_BITS;
irsend.sendMidea(code, bits, repeat);
break;
#endif
#if SEND_MAGIQUEST
case MAGIQUEST: // 35
if (bits == 0)
bits = MAGIQUEST_BITS;
irsend.sendMagiQuest(code, bits, repeat);
break;
#endif
#if SEND_LASERTAG
case LASERTAG: // 36
if (bits == 0)
bits = LASERTAG_BITS;
irsend.sendLasertag(code, bits, repeat);
break;
#endif
#if SEND_CARRIER_AC
case CARRIER_AC: // 37
if (bits == 0)
bits = CARRIER_AC_BITS;
irsend.sendCarrierAC(code, bits, repeat);
break;
#endif
#if SEND_MITSUBISHI2
case MITSUBISHI2: // 39
if (bits == 0)
bits = MITSUBISHI_BITS;
repeat = std::max(repeat, (uint16_t) MITSUBISHI_MIN_REPEAT);
irsend.sendMitsubishi2(code, bits, repeat);
break;
#endif
}
sendReqCounter++;
// Release the lock.
ir_lock = false;

// Indicate that we sent the message.
debug("Sent the IR message.");
debug("Type: " + String(ir_type));
switch (ir_type) {
case KELVINATOR:
case PRONTO:
case RAW:
case GLOBALCACHE:
debug("Code: ");
debug(code_str);
debug("Repeats: " + String(repeat));
// Confirm what we were asked to send was sent.

break;
default:
debug("Code: 0x" + uint64ToString(code, 16));
debug("Bits: " + String(bits));
debug("Repeats: " + String(repeat));
}}

void setColor(int inR, int inG, int inB) {
analogWrite(redPin, inR);
analogWrite(greenPin, inG);
analogWrite(bluePin, inB);

Serial.println("Setting LEDs:");
Serial.print("r: ");
Serial.print(inR);
Serial.print(", g: ");
Serial.print(inG);
Serial.print(", b: ");
Serial.println(inB);
}

void reconnect() {
// Loop until we're reconnected
int failedCnt = 0;
while (!client.connected()) {
Serial.print("Attempting MQTT connection...");
// Attempt to connect
if (client.connect(SENSORNAME, mqtt_user, mqtt_password)) {
Serial.println("connected");
//client.subscribe(light_set_topic);
client.subscribe(DEVICE_LED_COMMAND_TOPIC);
client.subscribe(DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC);
client.subscribe(DEVICE_LED_RGB_COMMAND_TOPIC);
client.subscribe(DEVICE_DEVICE_COMMAND_TOPIC);
setColor(0, 0, 0);
registerSensors();
sendAllState();
} else {
Serial.print("failed, rc=");
Serial.print(client.state());
Serial.println(" try again in 5 seconds");
failedCnt++;
if(failedCnt > 10){ // if failed to connect more than 10 times reset device to try and fix
Serial.println("Restarting device");
ESP.restart();
}
// Wait 5 seconds before retrying
delay(5000);
}
}
}


bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


void loop() {

server.handleClient();

ArduinoOTA.handle();

if (!client.connected()) {
reconnect();
}
client.loop();

//PIR CODE
long now = millis();
pirValue = digitalRead(PIRPIN); //read state of the
if (pirOldValue != pirValue) {
// Wait for 2 full triggers before sending a detect to try and debounce.
// For some reason there are a lot of false triggers and the default trigger
// time is 2000ms for the RCWL modules so wait for 2 triggers to reduce false reports
if (now - pirTimer > 4100) {
if (pirValue == LOW) {
motionStatus = false;
} else {
motionStatus = true;
}
sprintf(message_buff, "%s", (motionStatus) ? MQTT_ON_CMD : MQTT_OFF_CMD);
sendState(DEVICE_PIR_STATE_TOPIC, message_buff);
pirTimer = now;
pirOldValue = pirValue;
}
} else {
pirTimer = now;
}

//Button CODE
long now1 = millis();
BTNValue = digitalRead(BTNPIN); //read state of the button
if (BTNOldValue != BTNValue) {
if (now1 - BTNTimer > 100) { //debuncing
if (BTNValue == LOW) {
BTNStatus = false;
} else {
BTNStatus = true;
}
sprintf(message_buff, "%s", (BTNStatus) ? MQTT_ON_CMD : MQTT_OFF_CMD);
sendState(DEVICE_BUTTON_STATE_TOPIC, message_buff);
BTNTimer = now1;
BTNOldValue = BTNValue;
}
} else {
BTNTimer = now1;
}

float newTempValue = dht.readTemperature(true);
float newHumValue = dht.readHumidity();

if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
tempValue = newTempValue;
dtostrf(tempValue, 4, 2, str_buff);
sprintf(message_buff, "%s", str_buff);
sendState(DEVICE_TEMP_STATE_TOPIC, message_buff);
}

if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
humValue = newHumValue;
dtostrf(humValue, 4, 2, str_buff);
sprintf(message_buff, "%s", str_buff);
sendState(DEVICE_HUMIDITY_STATE_TOPIC, message_buff);
}

int newLDR = analogRead(LDRPIN);

if (checkBoundSensor(newLDR, LDR, diffLDR)) {
LDR = newLDR;
sprintf(message_buff, "%d", LDR);
sendState(DEVICE_LDR_STATE_TOPIC, message_buff);
}
delay(100);
}
