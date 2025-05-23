#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// MAX30102
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
double avered = 0, aveir = 0;
double sumirrms = 0, sumredrms = 0;
int i = 0;
int Num = 200;
double ESpO2 = 95.0;
double FSpO2 = 0.7;
double frate = 0.95;

float kalibrasi_suhuobj = 3.5;
int kalibrasi_bpm = 10;

#define TIMETOBOOT 3000
#define SAMPLING 5
#define FINGER_ON 30000
#define MINIMUM_SPO2 0.0
#define MINIMUM_BPM 0.0

// WiFi + MQTT
const char* ssid = "KELABANG";
const char* password = "abcd1234";
const char* mqtt_server = "iot.digitalasistensi.com";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE 200
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      client.subscribe("inTopic");
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED gagal"));
    while (true);
  }

  // MLX90614
  if (!mlx.begin()) {
    Serial.println("MLX90614 gagal. Periksa koneksi.");
    while (true);
  }

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 tidak ditemukan!");
    while (true);
  }
  particleSensor.setup(0x7F, 4, 2, 200, 411, 16384);
  particleSensor.enableDIETEMPRDY();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  uint32_t irValue, redValue;
  double fred, fir, SpO2 = 0;

  particleSensor.check();
  while (particleSensor.available()) {
    irValue = particleSensor.getFIFOIR();
    redValue = particleSensor.getFIFORed();

    if (checkForBeat(redValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    if (irValue < FINGER_ON) beatsPerMinute = MINIMUM_BPM;

    i++;
    fred = (double)redValue;
    fir = (double)irValue;
    avered = avered * frate + fred * (1.0 - frate);
    aveir = aveir * frate + fir * (1.0 - frate);
    sumredrms += (fred - avered) * (fred - avered);
    sumirrms += (fir - aveir) * (fir - aveir);

    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      SpO2 = -23.3 * (R - 0.4) + 100;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
    }

    float ambientTemp = mlx.readAmbientTempC();
    float objectTemp = mlx.readObjectTempC();
    float objectTempCal = objectTemp + kalibrasi_suhuobj;
    int beatAvgCal = beatAvg + kalibrasi_bpm;

    if (millis() - lastMsg > 2000) {
      lastMsg = millis();

      // Format CSV: sensor_name,bpm,spo2,temp_ambient,temp_object,status
      snprintf(msg, MSG_BUFFER_SIZE,
        "skripsi_adel,%d,%.2f,%.2f,%.2f,%s",
        (irValue < FINGER_ON ? 0 : beatAvg),
        (irValue < FINGER_ON ? 0.0 : ESpO2),
        ambientTemp,
        objectTempCal,
        (irValue < FINGER_ON ? "No finger" : "OK"));

      Serial.println(msg);
      client.publish("skripsi_adel/data", msg);


      // OLED
      display.clearDisplay();
      display.setCursor(2, 0);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.print("BPM : "); display.println(beatAvgCal);
      display.println("");
      display.print("SpO2: "); display.print(ESpO2, 1); display.println("%");
      display.println("");
      display.print("Suhu: "); display.print(objectTempCal, 1); display.println("C");
      display.display();
    }

    particleSensor.nextSample();
  }
}
