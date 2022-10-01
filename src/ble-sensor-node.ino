// Code for all one complete sensor node
#include <Particle.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>	
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_VEML6070.h>
#include <sps30.h>	// https://github.com/paulvha/sps30 , line 190 edited

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

BH1750 bh;
Adafruit_BME280 bme;
SCD30 airSensor;
SPS30 sps30;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
uint16_t ADC_VALUE = 0;
SystemSleepConfiguration sleepConfig;
TCPClient serverClient;

const char* serviceUuid = "58F75BE1-6DF6-4273-9627-CA053E89771B";
const char* sensorMode  = "58F75BE2-6DF6-4273-9627-CA053E89771B";
const byte qwiicAddress = 0x30;
float dBnumber = 0.0;
int sensorErrorCount;
String deviceName = "Argon_1";
byte server[] = { 192, 168, 100, 100 };
int port = 8888;
bool singleMeasurement = false;
bool continuousMeasurement = false;

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDRESS 0x77
#define SP30_COMMS I2C_COMMS
#define AUTOCLEANINTERVAL 604800
#define TX_PIN 0
#define RX_PIN 0
#define DEBUG 0
#define COMMAND_GET_VALUE 0x05
#define COMMAND_NOTHING_NEW 0x99
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)


void initializeSensors();
JSONBufferWriter getSensorReadings(JSONBufferWriter writerData);
void qwiicTestForConnectivity();
void qwiicGetValue();
JSONBufferWriter readSPS30(JSONBufferWriter writerData);
void goSleep();
void syncClock();
void checkErrorReset();
void readPublishSensors();
bool measurementCommand();

// setup() runs once, when the device is first turned on.
void setup() {
	pinMode(D7,OUTPUT);
	Particle.connect();
	Wire.begin();
	Serial.begin();
	sensorErrorCount = 0;
	initializeSensors();

	BleUuid bleService(serviceUuid);
	BleCharacteristic modeCharacteristic("sensorMode", BleCharacteristicProperty::WRITE_WO_RSP, sensorMode, serviceUuid, onDataReceived, (void*)sensorMode);
	BLE.addCharacteristic(modeCharacteristic);
	BleAdvertisingData advData;
	advData.appendLocalName(deviceName);
	advData.appendServiceUUID(bleService);
	BLE.advertise(&advData);

	// Wait for background tasks and sensor initialization to finish
	delay(20s);
	sps30.sleep();
	sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER).ble();
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
	Serial.begin();
    sps30.wakeup();
    delay(10);
    sps30.start();
    delay(30s);

    // Sensor reading decision
	while (measurementCommand())
	{
		delay(6s);	// 5s + 1s due to clock on SCD30 being slightly slower
		readPublishSensors();
		singleMeasurement = false;
	}

	// Shut down
    sps30.stop();
    delay(10);
    sps30.sleep();
	WiFi.off();

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

	// Particulate sensor SPS30
	sps30.EnableDebugging(DEBUG);
	while (!sps30.begin(SP30_COMMS)) 
	{
		delay(1s);
		Serial.println("Trying to connect Particulate SPS30 Sensor");
		sensorErrorCount++;
		checkErrorReset();
	}
	sps30.SetAutoCleanInt(AUTOCLEANINTERVAL);
	sps30.reset();

	// Zio Qwiic Loudness Sensor Master
	qwiicTestForConnectivity();
	Serial.println("Zio Qwiic Loudness Sensor Master Awake");

	// VEML6070 UV Level Sensor
	uv.begin(VEML6070_1_T);
}

void readPublishSensors() 
{
	Serial.println("read & publish sensors start");
	// Get sensor readings & write to data buffer in memory
	char *dataString = (char *) malloc(500);
	JSONBufferWriter writerData(dataString, 499);
    writerData.beginArray();
	writerData.value(deviceName);
    writerData = getSensorReadings(writerData);

	// End sensor reading
    writerData.endArray();
	writerData.buffer()[std::min(writerData.bufferSize(), writerData.dataSize())] = 0;
	Serial.println("taken sensor reading");

	// Publish collated sensor data string
	Serial.print("Collated:");
	Serial.println(writerData.dataSize());
	Serial.println(dataString);
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
	bh.make_forced_measurement();					// default 4 decimal place
	writerData.value(bh.get_light_level());

	// Peak Sound Sensor (SPARKFUN SEN-15892)
	qwiicGetValue();
	writerData.value(dBnumber);

	// UV Sensor (VEML 6070)
	writerData.value(uv.readUV());					// default whole numbers

	// Pressure, Temperature, Humidity Sensor (BME280)
	writerData.value(bme.readPressure());			// default 2 decimal place
	writerData.value(bme.readTemperature());		// default 2 decimal place
	writerData.value(bme.readHumidity());			// default 4 decimal place

	// Particulate Sensor (SPS30)
	writerData = readSPS30(writerData);				// default 5 decimal place
	sps30.sleep();

	// CO2 Sensor (SCD30)
	writerData.value(airSensor.getCO2());			// default whole numbers

	return writerData;
}

bool measurementCommand()
{
	return (singleMeasurement || continuousMeasurement);
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

JSONBufferWriter readSPS30(JSONBufferWriter writerData) 
{
	uint8_t ret;
	struct sps_values val;

	// loop to get data
	do 
	{
		ret = sps30.GetValues(&val);
	} while (ret != SPS30_ERR_OK);

	// pm2 refers to PM2.5 reading
	writerData.value(val.MassPM1);
	writerData.value(val.MassPM2);
	writerData.value(val.MassPM4);
	writerData.value(val.MassPM10);
	
	return writerData;
}

void checkErrorReset() {
	delay(5s);
	if (sensorErrorCount == 5) 
	{
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

	// Single measurement mode
	if (data[0] == 0xff) 
	{
		singleMeasurement = true;
		continuousMeasurement = false;
		return;
	}

	// Continuous measurement mode
	if (data[0] == 0x00) 
	{
		continuousMeasurement = true;
		return;
	}
}

/* HELPER FUNCTIONS END */