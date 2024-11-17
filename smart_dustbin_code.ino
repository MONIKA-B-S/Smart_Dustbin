#include <HX711.h>           // Library for the load cell
#include <DHT.h>             // Library for the DHT11 sensor

// Pin definitions
#define DHTPIN 4             // Pin connected to DHT11 data pin (GPIO 4 = D2 on ESP8266)
#define DHTTYPE DHT11        // DHT 11 sensor type
#define LOADCELL_DOUT_PIN 12 // HX711 DOUT pin (GPIO 12 = D6 on ESP8266)
#define LOADCELL_SCK_PIN 14  // HX711 SCK pin (GPIO 14 = D5 on ESP8266)
#define BATTERY_PIN A0       // Pin connected to battery voltage monitoring (A0 on ESP8266)
#define TRIG_PIN 5           // Ultrasonic sensor trigger pin (GPIO 5 = D1 on ESP8266)
#define ECHO_PIN 13          // Ultrasonic sensor echo pin (GPIO 13 = D7 on ESP8266)

// Load cell calibration factor and setup
HX711 scale;
float calibration_factor = -300;  // Set this value by calibration
long zero_factor;                    // Zero factor after taring the scale

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  
  // Initialize DHT sensor
  dht.begin();

  // Initialize HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();            // Reset the scale to no scaling
  scale.tare();                 // Reset the scale to zero

  zero_factor = scale.read_average();  // Get the initial zero factor

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Setup complete, start reading values...");
}

void loop() {
  // Measure load cell data
  float weight = scale.get_units(5);   // Average over 5 readings
  Serial.print("Weight: ");
  Serial.print(weight, 1);             // Print weight with one decimal place
  Serial.println(" kg");

  // Measure DHT11 temperature and humidity
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
  }

  // Measure battery voltage (solar panel powered battery)
  int batteryValue = analogRead(BATTERY_PIN);
  float batteryVoltage = batteryValue * (3.3 / 1024.0) * 2;  // Assuming voltage divider with 2:1 ratio
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  // Measure distance using ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);  // Get the echo duration
  distance = duration * 0.034 / 2;     // Convert to distance in cm
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Delay between readings
  delay(2000);
}
