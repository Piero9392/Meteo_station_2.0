/*
LoRa Ra-02 wiring:
MOSI:  GPIO 23
MISO:  GPIO 19
SCK:   GPIO 18
NSS:   GPIO 5
RESET: GPIO 14
D100:  GPIO 2

BME680 sensor wiring:
SDA:   GPIO 21
SCL:   GPIO 22
*/

// Include necessary libraries
#include "bsec.h"  // Include the BME680 sensor library
#include <SPI.h>   // Include the SPI library for LoRa
#include <LoRa.h>  // Include the LoRa library

// Pin LoRa configurations
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 14
#define DI0 2

// LoRa configuration parameters
#define BAND 433E6
#define SPREADING_FACTOR 12 // Match spreading factor with transmitter
#define SIGNAL_BANDWIDTH 125E3 // Match bandwidth with transmitter
#define CODING_RATE 5 // Match coding rate with transmitter
#define PREAMBLE_LENGTH 8 // Match preamble length with transmitter
#define SYNC_WORD 0x6E // Set custom sync word
#define TX_POWER 20 // Set maximum transmission power in dBm (Ra-02 supports up to 20dBm)

// Pin configurations for BME680
#define SDA 21
#define SCL 22
#define BME680_ADDRESS 0X77

Bsec iaqSensor; // BME680 sensor object

String LoRaMessage = ""; // String to store LoRa message

unsigned long previousMillis = 0;
const long intervalDataSend = 1000;

// Function to initialize BSEC virtual sensors
void bsecVirtualSensor() {
  // Configure BME680 virtual sensors
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP); // Update sensor subscription
}

// Function to calculate dew point
double dewPointFunction(double celsius, double humidityOut)  {
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidityOut * 0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}

// Function to calculate altitude
double altitudeFunction(double pressureOut)  {
  double t = 288.15; // Standard temperature at sea level (T0)
  double l = 0.0065; // Temperature lapse rate (L)
  double p = 1013.25; // Pressure at sea level (P0)
  double r = 8.314462618; // Universal gas constant (R)
  double g = 9.80665; // Acceleration due to gravity (G)
  double m = 0.0289644; // Molar mass of Earth's air (M0)
  double res = pow((pressureOut / p), ((r * l) / (g * m)));
  double Alt = t / l * (1-res);
  return Alt;
}

// Function to initialize LoRa communication
void startLora() {
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed");
    while (1)
      ;
  }
  // Adjust LoRa parameters to match transmitter settings
  LoRa.setSpreadingFactor(SPREADING_FACTOR); // Match spreading factor with transmitter
  LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH); // Match bandwidth with transmitter
  LoRa.setCodingRate4(CODING_RATE); // Match coding rate with transmitter
  LoRa.setPreambleLength(PREAMBLE_LENGTH); // Match preamble length with transmitter
  LoRa.setSyncWord(SYNC_WORD); // Set custom sync word
  LoRa.setTxPower(TX_POWER, PA_OUTPUT_PA_BOOST_PIN); // Set maximum transmission power in dBm (Ra-02 supports up to 20dBm). Set transmission power mode

  Serial.println("LoRa 433 MHz Receiver");
  Serial.println("LoRa initializing OK");
}

// Function to send data via LoRa module
void sendDataLora() {
  if (iaqSensor.run()) { // Read sensor data
    // Get sensor data
    int temperatureOut = iaqSensor.temperature;
    int humidityOut = iaqSensor.humidity;
    int pressureOut = iaqSensor.pressure / 100;
    int dewPointOut = dewPointFunction(temperatureOut, humidityOut);
    int gasOut = iaqSensor.gasResistance / 1000;
    int iaqOut = iaqSensor.iaq;
    int co2Out = iaqSensor.co2Equivalent;
    float vocOut = iaqSensor.breathVocEquivalent;
    int altitudeOut = altitudeFunction(pressureOut);

    // Format sensor data into a string for LoRa transmission
    LoRaMessage = String(temperatureOut) + "/" + String(humidityOut) + "&" + String(pressureOut) + "#" + String(dewPointOut) + "@" + String(gasOut) + "$" + String(iaqOut) + "^" + String(co2Out) + "!" + String(vocOut) + "<" + String(altitudeOut);
    LoRa.beginPacket(); // Begin LoRa transmission
    LoRa.print(LoRaMessage); // Send LoRa message
    LoRa.endPacket(); // End LoRa transmission

    // Print sensor data to serial monitor
    Serial.println("Environmental data from Sensor OUTSIDE:");
    Serial.print("Temperature    | ");
    Serial.print(temperatureOut);
    Serial.println(" \u00B0C");
    Serial.print("Humidity       | ");
    Serial.print(humidityOut);
    Serial.println(" %");
    Serial.print("Pressure       | ");
    Serial.print(pressureOut);
    Serial.println(" hPa");
    Serial.print("Dew Point      | ");
    Serial.print(dewPointOut);
    Serial.println(" \u00B0C");
    Serial.print("Gas resistance | ");
    Serial.print(gasOut);
    Serial.println(" kOhm");
    Serial.print("IAQ            | ");
    Serial.print(iaqOut);
    Serial.println(" PPM");
    Serial.print("CO2            | ");
    Serial.print(co2Out);
    Serial.println(" PPM");
    Serial.print("VOC            | ");
    Serial.print(vocOut);
    Serial.println(" PPM");
    Serial.print("Altitude       | ");
    Serial.print(altitudeOut);
    Serial.println(" m");
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200); // Initialize serial communication
  Wire.begin(SDA, SCL); // Initialize I2C communication for BME680
  SPI.begin(SCK, MISO, MOSI, SS); // Initialize SPI communication for LoRa
  iaqSensor.begin(BME680_ADDRESS, Wire); // Initialize BME680 sensor
  startLora(); // Initialize LoRa communication
  bsecVirtualSensor(); // Configure BME680 virtual sensors
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalDataSend) {
    previousMillis = currentMillis;
    sendDataLora(); // Transmit data via LoRa periodically
  }
}
