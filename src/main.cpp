#include <iostream>
#include <queue>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

using namespace std;

// connect WIFI
const char *ssid = "VjumpKunG";
const char *password = "Kungjump-1";

// temp_color
#define RED_TEMP 2
#define GREEN_TEMP 0
#define BLUE_TEMP 4

// humidity_color
#define RED_HUMID 13
#define GREEN_HUMID 12
#define BLUE_HUMID 14

// CO_color
#define RED_CO 5
#define GREEN_CO 18
#define BLUE_CO 19

#define MQ_9 36
#define DHTPIN 15

DHT dht(DHTPIN, DHT11);

// R G B
int temp_color[3] = {0, 0, 0};
int humidity_color[3] = {0, 0, 0};
int CO_color[3] = {0, 0, 0};

// temp humid co
bool status[3] = {false, false, false};

struct data
{
  float temperature;
  float humidity;
  int CO;
};

queue<data> q;

void READ_AirQuality(void *param);
void POST_AirQuality(void *param);
void GET_LEDStatus(void *param);
void UpdateLED();
void Connect_Wifi();

void setup()
{
  Serial.begin(115200);

  pinMode(RED_TEMP, OUTPUT);
  pinMode(BLUE_TEMP, OUTPUT);
  pinMode(GREEN_TEMP, OUTPUT);

  pinMode(RED_HUMID, OUTPUT);
  pinMode(BLUE_HUMID, OUTPUT);
  pinMode(GREEN_HUMID, OUTPUT);

  pinMode(RED_CO, OUTPUT);
  pinMode(BLUE_CO, OUTPUT);
  pinMode(GREEN_CO, OUTPUT);

  pinMode(MQ_9, INPUT);

  dht.begin();

  Connect_Wifi();

  xTaskCreatePinnedToCore(POST_AirQuality, "POST_AirQuality", 10000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(GET_LEDStatus, "GET_LEDStatus", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(READ_AirQuality, "READ_AirQuality", 10000, NULL, 1, NULL, 1);
}

void loop()
{
}

void READ_AirQuality(void *param)
{
  while (1)
  {
    float humidity = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(humidity) || isnan(t))
    {
      Serial.println("Failed to read from DHT sensor!");
      continue;
    }

    float temperature = dht.computeHeatIndex(t, humidity, false);
    int co = analogRead(MQ_9);

    Serial.println("Pushing data to queue");
    q.push(data{temperature, humidity, co});

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void POST_AirQuality(void *param)
{
  while (1)
  {
    Serial.print("");
    while (!q.empty())
    {
      const String url = "https://airquality.zeqa.net/air_quality/update_data/";
      HTTPClient http;
      http.begin(url);
      http.addHeader("Content-Type", "application/json");

      String json;
      DynamicJsonDocument doc(2048);
      data d = q.front();
      doc["temperature"] = d.temperature;
      doc["humidity"] = d.humidity;
      doc["co"] = d.CO;
      serializeJson(doc, json);

      Serial.printf("Posting temperature: %.2f, humidity: %.2f, co: %d\n", d.temperature, d.humidity, d.CO);

      int httpResponseCode = http.POST(json);
      if (httpResponseCode == 200)
      {
        String payload = http.getString();
        deserializeJson(doc, payload);

        temp_color[0] = doc["temperature_R"].as<int>();
        temp_color[1] = doc["temperature_G"].as<int>();
        temp_color[2] = doc["temperature_B"].as<int>();
        humidity_color[0] = doc["humidity_R"].as<int>();
        humidity_color[1] = doc["humidity_G"].as<int>();
        humidity_color[2] = doc["humidity_B"].as<int>();
        CO_color[0] = doc["CO_R"].as<int>();
        CO_color[1] = doc["CO_G"].as<int>();
        CO_color[2] = doc["CO_B"].as<int>();

        UpdateLED();
        Serial.println("POST Done");

        q.pop();
      }
      else
      {
        Serial.printf("POST Error %d\n", httpResponseCode);
      }
    }
  }
}

void GET_LEDStatus(void *param)
{
  while (1)
  {
    Serial.println("Getting LED status");

    const String url = "https://airquality.zeqa.net/air_quality/get_led_status/";
    DynamicJsonDocument doc(2048);
    HTTPClient http;
    http.begin(url);

    int httpResponseCode = http.GET();
    if (httpResponseCode == 200)
    {
      String payload = http.getString();
      deserializeJson(doc, payload);

      status[0] = doc["temperature"].as<bool>();
      status[1] = doc["humidity"].as<bool>();
      status[2] = doc["co"].as<bool>();

      UpdateLED();
      Serial.println("GET Done");

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else
    {
      Serial.printf("GET Error %d\n", httpResponseCode);
    }
  }
}

void UpdateLED()
{
  analogWrite(RED_TEMP, status[0] ? temp_color[0] : 0);
  analogWrite(GREEN_TEMP, status[0] ? temp_color[1] : 0);
  analogWrite(BLUE_TEMP, status[0] ? temp_color[2] : 0);

  analogWrite(RED_HUMID, status[1] ? humidity_color[0] : 0);
  analogWrite(GREEN_HUMID, status[1] ? humidity_color[1] : 0);
  analogWrite(BLUE_HUMID, status[1] ? humidity_color[2] : 0);

  analogWrite(RED_CO, status[2] ? CO_color[0] : 0);
  analogWrite(GREEN_CO, status[2] ? CO_color[1] : 0);
  analogWrite(BLUE_CO, status[2] ? CO_color[2] : 0);
}

void Connect_Wifi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("OK! IP=");
  Serial.println(WiFi.localIP());
}