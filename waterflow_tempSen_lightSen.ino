//GY-30 Light intensity sensor. Connection pin : D1 = SCL (GY-30) and D2 = SOA (GY-30)
#include <Wire.h>
const int GY30_ADDR = 0x23; // I2C address of GY-30 sensor


//YF-S201 Water flow sensor, connection pin D5 and D6
int flow1_sensorPin = D5;
int flow2_sensorPin = D6;
volatile long pulse1, pulse2;
unsigned long lastTime1, lastTime2;
float volume1, volume2;


//18B20 Temp. Sensor, connection pin D7
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13 // Data wire is plugged into port D7 on the NodeMCU

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 
int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address


void setup() {
  Serial.begin(9600);
 
  //GY-30
  Wire.begin();
  delay(200);
  
  //YF-S201
  pinMode(flow1_sensorPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(flow1_sensorPin), increase1, RISING);
  pinMode(flow2_sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flow2_sensorPin), increase2, RISING);

  //18B20 Temp. sensor
  Serial.begin(9600);  // start serial port
  sensors.begin(); // Start up the library
  numberOfDevices = sensors.getDeviceCount();  // Grab a count of devices on the wire
  Serial.print("Locating devices...");  // locate devices on the bus
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
		} else {
		  Serial.print("Found ghost device at ");
		  Serial.print(i, DEC);
		  Serial.print(" but could not detect address. Check power and cabling");
		}
  }


}

void loop() {
  // Request 2 bytes of data from GY-30 sensor
  Wire.beginTransmission(GY30_ADDR);
  Wire.write(0x10); // 1 [Lux] resolution mode
  Wire.endTransmission();
  delay(200);

  // Read 2 bytes of data from GY-30 sensor
  Wire.requestFrom(GY30_ADDR, 2);
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int lux = (highByte << 8) + lowByte;

    // Print the light intensity in lux
    Serial.print("Light intensity: ");
    Serial.print(lux);
    Serial.println(" lux");  
  }

  delay(1000); // Wait for 1 second before taking the next reading

  //YF-S201 start
  volume1 = 2.663 * pulse1 / 1000 * 30;
  if (millis() - lastTime1 > 2000) {
    pulse1 = 0;
    lastTime1 = millis();
  }
  //Serial.print(volume1);
  //Serial.println(" L/m");
  //delay(500);

  volume2 = 2.663 * pulse2 / 1000 * 30;
  if (millis() - lastTime2 > 2000) {
    pulse2 = 0;
    lastTime2 = millis();
  }
  //Serial.print(volume2);
  //Serial.println(" L/m");
  delay(500);
  Serial.println("Flow 1 = " + String(volume1) + " L/min, " + "Flow 2 = " + String(volume2) + " L/min." );

  //YF-S201 end


  //18B20 Start
  sensors.requestTemperatures(); // Send the command to get temperatures
  
  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++) {
    
    if(sensors.getAddress(tempDeviceAddress, i)){// Search the wire for address
		
	
		Serial.print("Temperature for device: "); 	// Output the device ID
		Serial.println(i+1, DEC);
    float tempC = sensors.getTempC(tempDeviceAddress);    // Print the data
    Serial.print("Temp C: ");
    Serial.print(tempC);
    Serial.print(" Temp F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
    delay(1000);
    } 	
  }

  //18B20 end

}

//function for YF-S201
ICACHE_RAM_ATTR void increase1() {
  pulse1++;
}
ICACHE_RAM_ATTR void increase2() {
  pulse2++;
}

// function to print a device address for 18B20
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}