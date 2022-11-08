//#define DEBUG 1
//#define DHT_DEBUG 1
//#define ENABLE_THINGSPEAK 1

/* Exponential Moving Average - Alpha parameter */
/*This is in percentage. Should be between 0-99*/
#ifndef EMA_ALPHA_TEM
  #define EMA_ALPHA_TEM 10
#endif
#ifndef EMA_ALPHA_HUM
  #define EMA_ALPHA_HUM 5
#endif

#include "DHT.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#ifdef ENABLE_THINGSPEAK
  #include <ThingSpeak.h>
#endif

#include "config.h"

struct measurement {
  float raw;
  float average;
  float current;
};
struct measurement temperature;
struct measurement humidity;
byte iteration = 0;

int status = WL_IDLE_STATUS;

#define BUFFER_SIZE 100
char buf[BUFFER_SIZE];


// Initialize DHT sensor.
// DHT
#define DHTPIN 0 // board pin D3
#define DHTTYPE DHT22
#define MIN_INTERVAL 2500
DHT dht(DHTPIN, DHTTYPE);

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup()
{
  Serial.begin(115200);
  // clear arduino serial monitor page
  DEBUG_OUT("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  DEBUG_OUT("\nESP8266 temperature and humidity sensor");
  DEBUG_OUT("Setup started");
  
  pinMode(LED_BUILTIN, OUTPUT);
  // turn off wemos LED
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  
  
  client.setServer(MQTT_SERVER_ADDRESS, MQTT_SERVER_PORT);
  #ifdef ENABLE_THINGSPEAK
    ThingSpeak.begin(wifiClient);
  #endif

  dht.begin();

  DEBUG_OUT("Initialized, reading initial values...");
  // initialize smoothing filter with first measurements
  delay(MIN_INTERVAL);
  dhtRead();
  temperature.average = temperature.raw;
  humidity.average = humidity.raw;
  
  delay(MIN_INTERVAL);
  DEBUG_OUT("Setup end");
}

void loop()
{
  startLoop();
  dhtRead();

  bool changed = false;
  
  ema_filter(temperature, EMA_ALPHA_TEM);
  if (isChanged(temperature)) {
    changed = true;
    temperature.current = round10(temperature.average);
    DEBUG_OUT("Temperature smoothed: " + String(temperature.current, 1) + " *C");
    publish("temperature", float2bytes(temperature.current, 1));
  }

  ema_filter(humidity, EMA_ALPHA_HUM);
  if (isChanged(humidity)) {
    changed = true;
    humidity.current = round10(humidity.average);
    DEBUG_OUT("Humidity smoothed: " + String(humidity.current, 1) + " %");
    publish("humidity", float2bytes(humidity.current, 1));
  }

  if (changed) {
    #ifdef ENABLE_THINGSPEAK
      ThingSpeak.setField(1, temperature.current);
      ThingSpeak.setField(2, humidity.current);
      ThingSpeak.writeFields(THINGSPEAK_CHANNEL, THINGSPEAK_API_KEY);
    #endif
  }
  
  endLoop();
  
  DEBUG_OUT("Measurements processed. Sleeping...");
  delay(SLEEP_TIME);
}

void startLoop()
{
  // Turn on onboard LED
  digitalWrite(LED_BUILTIN, LOW);
  iteration++;
}

void endLoop()
{
  disconnect();
  // Turn off onboard LED
  digitalWrite(LED_BUILTIN, HIGH);
}

void dhtRead()
{
  static float tem, hum;
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  startDhtRead:
  tem = dht.readTemperature();
  hum = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(tem)) {
    DEBUG_OUT("Failed to read from DHT sensor!");
    delay(5000);
    dht.begin();
    goto startDhtRead;
  }
  DEBUG_OUT("Humidity raw: " + String(hum, 1) + " %");
  DEBUG_OUT("Temperature raw: " + String(tem, 1) + " *C");
  temperature.raw = tem;
  humidity.raw = hum;
}

void ema_filter(measurement &m, byte alpha)
{
    m.average = (alpha * m.raw + (100 - alpha) * m.average) / 100;
    DEBUG_OUT("EMA average: " + String(m.average, 2));
}

bool isChanged(measurement m)
{
  return abs(m.average - m.current) >= 0.06;
}

float round10(float value)
{
  return round(value * 10.0) / 10.0;
}

void publish(String topic, const char* payload)
{
  reconnect();
  client.publish(getTopic(topic), payload, MQTT_MESSAGE_RETAIN);
}

// attempt to connect to WiFi network
void initWiFi()
{
  // Setting persistent to false will get SSID / password written to flash only if currently used values do not match what is already stored in flash.
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  DEBUG_OUT("[WiFi] Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  DEBUG_OUT("[WiFi] Connected, ip address: " + String(WiFi.localIP().toString().c_str()));
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    status = WiFi.status();
    if (status != WL_CONNECTED) {
      initWiFi();
    }
    DEBUG_OUT("[MQTT] Connecting to broker ...");
    if (client.connect(MQTT_CLIENT_ID, MQTT_CLIENT_USER, MQTT_CLIENT_PASSWORD)) {
      DEBUG_OUT("[MQTT] connected");
      return;
    } else {
      DEBUG_OUT("[MQTT] failed to connect: rc=" + String(client.state()));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void disconnect()
{
  // disconnect MQTT client, keepalive is shorter than pause between measurements
  if (client.connected()) {
    client.disconnect();
    DEBUG_OUT("[MQTT] disconnect");
  }
}

/**
 * Get topic as constant char pointer. Suitable for client.publish()
 */
const char* getTopic(String measurement)
{
  String topic = String(MQTT_TOPIC_PREFIX + measurement);

  return topic.c_str();
}

/**
 * Convert float value to char array. Suitable as payload for client.publish()
 */
const char* float2bytes(float value, int precision)
{
  String str = String(value, precision);
  str.toCharArray(buf, BUFFER_SIZE);

  return buf;
}

/**
 * Convert integer value to char array.
 */
const char* int2bytes(int value)
{
  String str = String(value);
  str.toCharArray(buf, BUFFER_SIZE);

  return buf;
}
