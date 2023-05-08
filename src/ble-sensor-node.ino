// Code for all one complete sensor node
#include <Particle.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>	
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_VEML6070.h>
#include <SensirionI2CSen5x.h>	

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

/* GLOBAL DECLARATIONS */
int sensorErrorCount;
void initializeSensors();
void checkErrorReset();
void readPublishSensors();
bool measurementCommand();
SystemSleepConfiguration sleepConfig;
JSONBufferWriter getSensorReadings(JSONBufferWriter writerData);
// TCP Client stuff
TCPClient serverClient;
String deviceName = "NodeC";
byte server[] = { 192, 168, 100, 100 };
int port = 8888;
// BLE Stuff
const char* serviceUuid = "58F75BE1-6DF6-4273-9627-CA053E89771B";
const char* sensorMode  = "58F75BE2-6DF6-4273-9627-CA053E89771B";
bool measureCommand = false;
bool sensorOffAfter = true;
bool sensorOn = false;
// BH1750 Lux Sensor
BH1750 bh;
// BME280 PTH Sensor
Adafruit_BME280 bme;
#define BME_ADDRESS 0x77
#define SEALEVELPRESSURE_HPA (1013.25)
// SCD30 CO2 Sensor
SCD30 airSensor;
// Particulate sensor SEN55
SensirionI2CSen5x sen5x;
// VEML6070 UV Level Sensor
Adafruit_VEML6070 uv = Adafruit_VEML6070();
// Zio Qwiic Loudness Sensor
uint16_t ADC_VALUE = 0;
const byte qwiicAddress = 0x38;
float dBnumber = 0.0;
float adcNumber = 0.0;
void qwiicTestForConnectivity();
void qwiicGetValue();
#define COMMAND_GET_VALUE 0x05
#define COMMAND_NOTHING_NEW 0x99
/* END GLOBAL DECLARATIONS */


// setup() runs once, when the device is first turned on.
void setup() {
	pinMode(D7,OUTPUT);
	Wire.begin();
	Serial.begin();
	Serial1.begin(115200);
	sensorErrorCount = 0;
	delay(10s);
	Serial.println("Setup start");
	initializeSensors();

	// BLE
	BleUuid bleService(serviceUuid);
	BleCharacteristic modeCharacteristic("sensorMode", BleCharacteristicProperty::WRITE_WO_RSP, sensorMode, serviceUuid, onDataReceived, (void*)sensorMode);
	BLE.addCharacteristic(modeCharacteristic);
	BleAdvertisingData advData;
	advData.appendLocalName(deviceName);
	advData.appendServiceUUID(bleService);
	BLE.advertise(&advData);

	// Wait for background tasks and sensor initialization to finish
	delay(20s);
	sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER).ble();
	Serial.println("Setup end");
}

// loop() runs over and over again, as quickly as it can execute. Main Program flow goes here!
void loop() {

    // Sleep until BLE activity wake
    System.sleep(sleepConfig);
    digitalWrite(D7,HIGH);

	// Listen for sensor measurement command, timeout in ms
	waitFor(measurementCommand, 20000);
	if (!measurementCommand())
	{
		// Skip activating sensors & go back to sleep
		goto endOfLoopIteration;
	}

	// Sensor data reading startup
	WiFi.on();
	WiFi.connect();
	if (!sensorOn)
	{
		// Wake up components that sleep
		sen5x.startMeasurement();
		Serial1.println("AT+GPS=1");
		Serial1.println("AT+GPSRD=20");
		sensorOn = true;
		delay(20s);
	}
	delay(10s);
	readPublishSensors();

	// Shut down
	measureCommand = false;
	if (sensorOffAfter)
	{
		sen5x.stopMeasurement();
		Serial1.println("AT+GPS=0");
		sensorOn = false;
	}
	WiFi.off();
	delay(5s);

	endOfLoopIteration:
		digitalWrite(D7,LOW);
}

/* HELPER FUNCTIONS START */

void initializeSensors()
{
	// BH1750 Lux Sensor
	while (!bh.begin()) 
	{
		delay(1s);
		Serial.println("Trying to connect BH1750 Lux Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	bh.set_sensor_mode(BH1750::forced_mode_low_res);

	// BME280 PTH Sensor, Recommended weather monitoring settings
	while (!bme.begin()) 
	{
		delay(1s);
		Serial.println("Trying to connect BME280 PTH Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

	// SCD30 CO2 Sensor
	while (!airSensor.begin()) 
	{
		delay(1s);
		Serial.println("Trying to connect SCD30 CO2 Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	airSensor.setMeasurementInterval(5);
  	airSensor.setAutoSelfCalibration(true);

	// Particulate sensor SEN55
	sen5x.begin(Wire);
	float tempOffset = 0.0;
	sen5x.setTemperatureOffsetSimple(tempOffset);

	// Zio Qwiic Loudness Sensor Master
	qwiicTestForConnectivity();
	Serial.println("Zio Qwiic Loudness Sensor Master Awake");

	// VEML6070 UV Level Sensor
	uv.begin(VEML6070_1_T);

	// A9G GPS
	Serial1.println("AT+RST=1");
}

void readPublishSensors() 
{
	Serial.println("read & publish sensors start");
	// Get sensor readings & write to data buffer in memory
	// 		Data formatting is as follows:
	// 		[deviceId, sensorData(in JSON format)]
	char *dataString = (char *) malloc(1500);
	JSONBufferWriter writerData(dataString, 1499);
	writerData.beginArray().value(deviceName);
		writerData.beginObject();
		writerData = getSensorReadings(writerData);
		writerData.endObject();
	writerData.endArray();
	writerData.buffer()[std::min(writerData.bufferSize(), writerData.dataSize())] = 0;

	// Publish collated sensor data string
	Serial.print("Collated:");
	Serial.println(writerData.dataSize());
	Serial.println(dataString);
	waitUntil(WiFi.ready);
	serverClient.connect(server, port);
	serverClient.print(dataString);

	// Wait until server confirms data received, then end connection
	waitFor(serverClient.available, 5000);
	serverClient.stop();
	free(dataString);
}

JSONBufferWriter getSensorReadings(JSONBufferWriter writerData)
{
	// LUX Sensor (BH1750)
	bh.make_forced_measurement();
	writerData.name("lux").value(bh.get_light_level());

	// Peak Sound Sensor (SPARKFUN SEN-15892)
	qwiicGetValue();
	writerData.name("loudness").beginObject();
		writerData.name("adc").value(adcNumber);	
		writerData.name("db").value(dBnumber);
	writerData.endObject();

	// UV Sensor (VEML 6070)
	writerData.name("UV").value(uv.readUV());

	// Pressure, Temperature, Humidity Sensor (BME280)
	writerData.name("bme280").beginObject();
		writerData.name("P").value(bme.readPressure());
		writerData.name("T").value(bme.readTemperature());	
		writerData.name("H").value(bme.readHumidity());	
	writerData.endObject();

	// Particulate Sensor (SEN55)
	float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;
	sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
	writerData.name("sen55").beginObject();
		writerData.name("PM1p0").value(massConcentrationPm1p0);
		writerData.name("PM2p5").value(massConcentrationPm2p5);
		writerData.name("PM4p0").value(massConcentrationPm4p0);
		writerData.name("PM10p0").value(massConcentrationPm10p0);
		writerData.name("H").value(ambientHumidity);
		writerData.name("T").value(ambientTemperature);
		writerData.name("voc").value(vocIndex);
		writerData.name("nox").value(noxIndex);
	writerData.endObject();

	// CO2 Sensor (SCD30)
	writerData.name("scd30").beginObject();
		writerData.name("CO2").value(airSensor.getCO2());			// default whole numbers
		writerData.name("T").value(airSensor.getTemperature());
		writerData.name("H").value(airSensor.getHumidity());
	writerData.endObject();

	// A9G GPS
	while (Serial1.available())
	{
		// Clear incoming buffer
		char c = Serial1.read();
	}
	String gpsOutput = "";
	delay(12s);
	while (Serial1.available())
	{
		char c = Serial1.read();
		gpsOutput += c;
	}
	writerData.name("gps").value(gpsOutput);

	return writerData;
}

void qwiicGetValue()
{
	Wire.beginTransmission(qwiicAddress);
	Wire.write(COMMAND_GET_VALUE); // command for status
	Wire.endTransmission(); // stop transmitting //this looks like it was essential.
	Wire.requestFrom(qwiicAddress, 2); // request 1 bytes from slave device qwiicAddress

	while (Wire.available())
	{ 
		// slave may send less than requested
		uint8_t ADC_VALUE_L = Wire.read();
		uint8_t ADC_VALUE_H = Wire.read();
		ADC_VALUE=ADC_VALUE_H;
		ADC_VALUE<<=8;
		ADC_VALUE|=ADC_VALUE_L;
		dBnumber = (ADC_VALUE+83.2073) / 11.003; //emprical formula to convert ADC value to dB
	}
	return;
}

// qwiicTestForConnectivity() checks for an ACK from an Sensor. If no ACK
// program freezes and notifies user.
void qwiicTestForConnectivity()
{
	Wire.beginTransmission(qwiicAddress);
	//check here for an ACK from the slave, if no ACK don't allow change?
	if (Wire.endTransmission() != 0) 
	{
		Serial.println("Check connections. No slave attached.");
		while (1);
	}
	return;
}

bool measurementCommand() 
{
	return measureCommand;
}

void checkErrorReset() {
	delay(5s);
	if (sensorErrorCount == 5) 
	{
		Serial.println("reset");
		System.reset();
	}
	return;
}

// Static function for handling Bluetooth Low Energy callbacks
static void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) 
{
	// We're only looking for one byte
	if(len != 1) 
	{
		return;
	}

	// Measurement mode 1 (Shut down sensors after)
	if (data[0] == 0xff) 
	{
		measureCommand = true;
		sensorOffAfter = true;
		return;
	}

	// Measurement mode 2 (No shut down sensors after)
	if (data[0] == 0x00) 
	{
		measureCommand = true;
		sensorOffAfter = false;
		return;
	}
}

/* HELPER FUNCTIONS END */