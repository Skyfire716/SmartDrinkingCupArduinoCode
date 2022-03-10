#include <TimeLib.h>
#include <Arduino_LSM6DSOX.h>
#include <ArduinoBLE.h>
#include <WiFiNINA.h>
#include <Servo.h>

#define PROJECT_ROOT /home/user/SmartDrinkingCupAD5932Test
#define TO_STRING(s) #s
#define ABSOLUTE_PATH(root, relative_path) TO_STRING(root/relative_path)
#define RELATIVE_PATH(library) ABSOLUTE_PATH(PROJECT_ROOT, library)
#include RELATIVE_PATH(AD5932.h)
#include RELATIVE_PATH(HCSR04.h)
#include RELATIVE_PATH(SmartCupUUIDS.h)
#include RELATIVE_PATH(ntp.h)
#include RELATIVE_PATH(arduino_secrets.h)

#define CAPACITIVE 0
#define ULTRASONIC 1
#define LASER 2


//=====Config=====
//#define COMPILEFOR ULTRASONIC
#define COMPILEFOR CAPACITIVE
//#define COMPILEFOR LASER
#define WAIT_FOR_SERIAL 0
#define USEWIFI 0
#define USENTP 0
#define IMU_VALUE_BUFFER 10
#define DEBUG 1
#define NUMBER_OF_DRINK_EVENTS 4096
//=====END Config=====

//=====IMU Config=====
#define IMU_DEBUG 0
#define IMU_GYRO_DEBUG 0
#define IMU_ACCEL_DEBUG 0
#define SMART_CUP_POSITION_STATE_DEBUG 0
#define IMU_BLE_UPDATE_INTERVAL 250
//=====END IMU Config=====

//=====WIFI Config=====
#define AP 0
#define LP 0
#define WIFI_DEBUG 1
//=====END WIFI Config=====

//====NTP Config=====
#define NTPLocalPort 2390
//=====END NTP Config=====

//====Ultrasonic Conig=====
#if COMPILEFOR == ULTRASONIC
#define maxPouringDuration 30##000
#define pouringBtnPin 4
#define prevPercent 0.93
#define ULTRASONIC_DEBUG 1
#endif
//====END Ultrasonic Conig=====


//=====Laser Config=====
#if COMPILEFOR == LASER
#define LASER_PIN 0
#define LASER_DEBUG 1
#endif
//=====END Laser Config=====


//=====Capacitance CONFIG=====
#if COMPILEFOR == CAPACITIVE
#define AD5932_DEBUG 0
#define AD5932_SYNCOUT 8   //SCAN Status information (INPUT for user)
#define AD5932_MSBOUT 7
#define AD5932_INTERRUPT 6 //Rising Edge stops the scan
#define AD5932_CTRL 5      //Triggers Frequency Scan   Or triggers an frequency increment of the scan
#define AD5932_SDATA 11
#define AD5932_SCLK 13     //Data moved in on every SCLK falling Edge fpr 16 clock pulses
#define AD5932_FSYNC 10    //Acts as Chip Enable  (Chip Selected on LOW)   Keep low for every x multiple 16 bit On the last pull up again
#define AD5932_STANDBY 9   //Setting pin on HIGH puts chip into powersaving mode stopping all operations !!!first reset chip than put it in sleep mode!!! (Control register or INTERRUPT pin)
#define MEASURE_PIN 3
#define CAPACITIVE_DEBUG 1
#define AD5932CLOCK 50##000##000
#define SAMPLES_BEFORE_INC 1
#define DELAY_BETWEEN_SAMPLES 0
AD5932 *ad;
int sweepIncrements = 0;
struct __attribute__((__packed__)) AD5932_MEASURE_UPDATE {
  float frequency;
  float response;
};
struct AD5932_MEASURE_UPDATE measureUpdate;
#endif
//=====END Capacitance CONFIG=====


struct __attribute__((__packed__)) IMU_ACCEL_DATA {
  float Ax;
  float Ay;
  float Az;
};

struct __attribute__((__packed__)) IMU_GYRO_DATA {
  float Gx;
  float Gy;
  float Gz;
};

struct IMU_ACCEL_DATA imu_accel_data;
struct IMU_GYRO_DATA imu_gyro_data;
float meanACCEL[IMU_VALUE_BUFFER * 3];
float meanGYRO[IMU_VALUE_BUFFER * 3];
int imuIndex = 0;
volatile bool write_data_to_device = false;

struct __attribute__((__packed__)) DRINK_EVENT {
  char date[19];    //yyyy.mm.dd-HH:mm:ss
  float volume;
  int state;
};


struct DRINK_EVENT drinkEvents[NUMBER_OF_DRINK_EVENTS];
int currentDrinkEvent = 0;

#if USEWIFI
char *ssids[] = {SECRET_SSID, SECRET_SSID2};    // your network SSIDS (name)
char *pass[] = {SECRET_PASSWORD, SECRET_PASSWORD2};  // your network passwords (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
int credentialsIndex = 1;
const int timeZone = 1;     // Central European Time
#endif

#if USEWIFI && USENTP
ntp *ntpc;
#endif

#if COMPILEFOR == LASER
float mean_laser_distance[10];
int mean_laser_added = 0;
byte laserMeasuringInterval = 40;
unsigned long laser_update;
#endif


#if COMPILEFOR == ULTRASONIC
float mean_distance[10];
int mean_added = 0;

bool pouring = false;
bool assembly = false;
byte pouringBtnState = LOW;
byte measuringInterval = 20;
byte servoAngle = 0;
float servoSmooth = 0;
float servoPrev = 0;
byte min_angle = 0;
byte max_angle = 180;
HCSR04 *hcsr;
Servo ultrasonic_servo;
unsigned long ultrasonic_update;
unsigned long pouring_event;
#endif


BLEService MeasuringTypeService(UUID_MEASURING_TYPE);
BLECharacteristic MeasuringTypeCharacteristic(UUID_MEASURING_TYPE, BLERead, sizeof(byte) * 14);

BLEService IMUAccelService(UUID_IMU_ACCEL);
BLECharacteristic IMUAccelCharacteristic(UUID_IMU_ACCEL, BLERead | BLENotify, sizeof(IMU_ACCEL_DATA));

BLEService IMUGyroService(UUID_GYRO_IMU);
BLECharacteristic IMUGyroCharacteristic(UUID_GYRO_IMU, BLERead | BLENotify, sizeof(IMU_GYRO_DATA));

#if USEWIFI
BLEService WIFIService(UUID_WIFI_RELATED);
BLECharacteristic WIFICharacteristic(UUID_WIFI_RELATED, BLERead | BLENotify | BLEWrite, sizeof(int));
#endif

#if COMPILEFOR == CAPACITIVE
BLEService AD5932ConfigurationService(UUID_AD5932CONFIGURATION);
BLECharacteristic AD5932ConfigurationCharacteristic(UUID_AD5932CONFIGURATION, BLERead | BLEWrite | BLEWriteWithoutResponse, sizeof(AD5932_DATA_STRUCT));

BLEService AD5932TriggerService(UUID_AD5932TRIGGER);
BLECharacteristic AD5932TriggerCharacteristic(UUID_AD5932TRIGGER, BLEWrite | BLERead | BLENotify, sizeof(byte));

BLEService AD5932MeasureResultService(UUID_AD5932_MEASURE_RESULT);
BLECharacteristic AD5932MeasureResultCharacteristic(UUID_AD5932_MEASURE_RESULT, BLERead | BLENotify, sizeof(AD5932_MEASURE_UPDATE));
#endif

#if COMPILEFOR == LASER
BLEService LaserDistanceService(UUID_LASER_DISTANCE);
BLECharacteristic LaserDistanceCharacteristic(UUID_LASER_DISTANCE, BLERead | BLENotify, sizeof(double));

BLEService LaserControlService(UUID_LASER_CONTROL);
BLECharacteristic LaserControlCharacteristic(UUID_LASER_CONTROL, BLERead | BLEWrite, sizeof(byte));
#endif
//BLEService AD5932ReplyService(UUID_REPLY);
//BLECharacteristic AD5932ReplyCharacteristic(UUID_REPLY, BLERead | BLENotify, sizeof());

#if COMPILEFOR == ULTRASONIC
BLEService HCSR04Service(UUID_HCSR04_DISTANCE);
BLECharacteristic HCSR04Characteristic(UUID_HCSR04_DISTANCE, BLERead | BLENotify, sizeof(float));

BLEService HCSR04ControlService(UUID_HCSR04_CONTROL);
BLECharacteristic HCSR04ControlCharacteristic(UUID_HCSR04_CONTROL, BLEWrite | BLERead, sizeof(long));
#endif

void bleCentralConnected(BLEDevice central) {
#if DEBUG
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
#endif
}

void bleCentralDisconnected(BLEDevice central) {
#if DEBUG
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
#endif
}

void read_from_central_occured(BLEDevice central, BLECharacteristic characteristic)
{
#if DEBUG
  Serial.print("Central Read Value from ");
  Serial.println(characteristic.uuid());
#endif
#if COMPILEFOR == CAPACITIVE
  AD5932ConfigurationCharacteristic.writeValue(ad->getDataStructPointer(), 14);
#endif
}

void write_from_central_occured(BLEDevice central, BLECharacteristic characteristic)
{
#if DEBUG
  Serial.print("Central wrote to ");
  Serial.println(characteristic.uuid());
#endif
#if COMPILEFOR == CAPACITIVE
  if (characteristic.uuid() == UUID_AD5932CONFIGURATION) {
    const uint8_t* buffer = AD5932ConfigurationCharacteristic.value();
    //AD5932ConfigurationCharacteristic.readValue(buffer, 14);
    memcpy(ad->getDataStructPointer(), AD5932ConfigurationCharacteristic.value(), 14);
    double TBase = 1.0 / AD5932CLOCK * ad->get_multiplier();
    double TScan = (1 + ad->get_number_of_increments()) * TBase;
#if DEBUG && CAPACITIVE_DEBUG
    Serial.println("Central Wrote Value to Buffer");
    Serial.println(buffer[0], BIN);
    Serial.println(buffer[1], BIN);
    Serial.println("\n\n\n");
    ad->print_config();
    Serial.print("Scan Time: ");
    Serial.print(TScan, 9);
    Serial.println("ns");
    Serial.print(TScan * 1000, 9);
    Serial.println("µs");
    Serial.print((TScan * 1000) * 1000, 9);
    Serial.println("ms");
#endif
    write_data_to_device = true;
  }
  if (characteristic.uuid() == UUID_AD5932TRIGGER) {
    if (*characteristic.value() == 0x01) {
#if DEBUG && CAPACITIVE_DEBUG
      Serial.println("Trigger");
#endif
      Serial.print("Size of measureUpdate ");
      Serial.println(sizeof(measureUpdate));
      sweepIncrements = 0;
      ad->trigger();
    } else if (*characteristic.value() == 0x02) {
#if DEBUG && CAPACITIVE_DEBUG
      Serial.println("Reset");
#endif
      ad->reset();
    }
    byte b = 0;
    characteristic.writeValue(&b, sizeof(byte));
  }
#endif
#if  COMPILEFOR == ULTRASONIC
  if (characteristic.uuid() == UUID_HCSR04_CONTROL) {
    const uint8_t* buffer = characteristic.value();
    servoAngle = buffer[0];
    byte control = buffer[1];
    min_angle = buffer[2];
    max_angle = buffer[3];
    measuringInterval = (byte)(control & 0x7F);
#if DEBUG && ULTRASONIC_DEBUG
    Serial.println("WrittenTo HCSR04Control");
    Serial.print("MinAngle: ");
    Serial.println(min_angle);
    Serial.print("MaxAngle: ");
    Serial.println(max_angle);
    Serial.print("UpdateInterval: ");
    Serial.println(measuringInterval);
    Serial.print("ServoAngle ");
    Serial.println(servoAngle);
#endif
    //ultrasonic_servo.write(servo_angle);
    if ((control & 0x80) == 0x80) {
      //TODO ToggleServo
      assembly = !assembly;
      if (ultrasonic_servo.read() < max_angle) {
        //ultrasonic_servo.write(max_angle);
      } else {
        //ultrasonic_servo.write(min_angle);
      }
    }
  }
#endif
#if COMPILEFOR == LASER
  if (characteristic.uuid() == UUID_LASER_CONTROL) {
    const uint8_t* buffer = characteristic.value();
    laserMeasuringInterval = buffer[0];
#if DEBUG
    Serial.print("LaserMeasurnigInterval updated to:" );
    Serial.println(laserMeasuringInterval);
#endif
  }
#endif
}

void subscribe_occured(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("SomethingSubscribed to ");
  Serial.println(characteristic.uuid());
}

void unsubscribe_occured(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("SomethingUnsubscribed to ");
  Serial.println(characteristic.uuid());
}

unsigned long dataupdate;
bool measurablePosition = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#if WAIT_FOR_SERIAL
  while (!Serial) {}
#endif
  Serial.println("Smart Drinking Cup Starting...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

#if USEWIFI
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
#if DEBUG && WIFI_DEBUG
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssids[credentialsIndex]);
#endif
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssids[credentialsIndex], pass[credentialsIndex]);
    while (status != WL_CONNECTED) {
      delay(20);
    }
  }
#if DEBUG && WIFI_DEBUG
  Serial.println("Connected to wifi");
#endif
#if USEWIFI && USENTP
  setSyncProvider(getNtpTime);
#endif
  WiFi.disconnect();
#if DEBUG && WIFI_DEBUG
  Serial.println("Disconnected from WiFi");
#endif
#endif

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
  }


  // set advertised local name and service UUID:
  BLE.setLocalName("SmartDrinkingCup BLE");
  // add the characteristic to the service

  BLE.setEventHandler(BLEConnected, bleCentralConnected);
  BLE.setEventHandler(BLEDisconnected, bleCentralDisconnected);

  IMUAccelService.addCharacteristic(IMUAccelCharacteristic);
  IMUGyroService.addCharacteristic(IMUGyroCharacteristic);
  MeasuringTypeService.addCharacteristic(MeasuringTypeCharacteristic);



  // add service
  BLE.addService(IMUAccelService);
  BLE.addService(IMUGyroService);
  BLE.addService(MeasuringTypeService);

  IMUAccelCharacteristic.setEventHandler(BLESubscribed, subscribe_occured);
  IMUGyroCharacteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);

#if USEWIFI
  WIFIService.addCharacteristic(WIFICharacteristic);

  WIFICharacteristic.setEventHandler(BLESubscribed, subscribe_occured);
  WIFICharacteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);
  WIFICharacteristic.setEventHandler(BLEWritten, write_from_central_occured);
  WIFICharacteristic.setEventHandler(BLERead, read_from_central_occured);

  BLE.addService(WIFIService);
#endif

#if COMPILEFOR == CAPACITIVE
  byte init_data[14];
  for (int i = 0; i < 14; i++) {
    init_data[i] = 0x01;
  }
  ad = new AD5932(AD5932_FSYNC, AD5932_STANDBY, AD5932_INTERRUPT, AD5932_CTRL, AD5932_SYNCOUT, AD5932_MSBOUT);
  AD5932ConfigurationService.addCharacteristic(AD5932ConfigurationCharacteristic);
  AD5932TriggerService.addCharacteristic(AD5932TriggerCharacteristic);
  AD5932MeasureResultService.addCharacteristic(AD5932MeasureResultCharacteristic);
  BLE.addService(AD5932ConfigurationService);
  BLE.addService(AD5932TriggerService);
  BLE.addService(AD5932MeasureResultService);
  AD5932ConfigurationCharacteristic.setEventHandler(BLEWritten, write_from_central_occured);
  AD5932ConfigurationCharacteristic.setEventHandler(BLERead, read_from_central_occured);
  AD5932TriggerCharacteristic.setEventHandler(BLEWritten, write_from_central_occured);
  AD5932TriggerCharacteristic.setEventHandler(BLERead, read_from_central_occured);
  AD5932TriggerCharacteristic.setEventHandler(BLESubscribed, subscribe_occured);
  AD5932TriggerCharacteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);
  AD5932MeasureResultCharacteristic.setEventHandler(BLESubscribed, subscribe_occured);
  AD5932MeasureResultCharacteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);
  AD5932ConfigurationCharacteristic.writeValue(init_data, 14);
  ad->set_multiplier(x1);
  ad->set_start_frequency(50);
  ad->set_delta_frequency(50);
  ad->set_trigger_mode(EXTERNAL_INCREMENT);
  ad->set_waveform(TRIANGULAR);
  ad->set_increment_interval_mode(BY_MCLK_PERIODS);
  ad->set_number_of_increments(2);
  ad->set_increment_interval(500);
  ad->set_msbout(false);
  ad->set_syncselect(SYNC_AT_EOS);
  ad->set_syncout(false);
  ad->print_config();
  AD5932ConfigurationCharacteristic.writeValue(ad->getDataStructPointer(), 14);
  byte b = 0;
  AD5932TriggerCharacteristic.writeValue(&b, sizeof(byte));
#endif

#if COMPILEFOR == ULTRASONIC
  pinMode(pouringBtnPin, INPUT_PULLUP);
  ultrasonic_servo.attach(5);
  hcsr = new HCSR04(2, 3);
  HCSR04Service.addCharacteristic(HCSR04Characteristic);
  HCSR04ControlService.addCharacteristic(HCSR04ControlCharacteristic);
  BLE.addService(HCSR04Service);
  BLE.addService(HCSR04ControlService);
  HCSR04Characteristic.setEventHandler(BLESubscribed, subscribe_occured);
  HCSR04Characteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);
  HCSR04Characteristic.setEventHandler(BLERead, read_from_central_occured);
  HCSR04ControlCharacteristic.setEventHandler(BLEWritten, write_from_central_occured);
  long by = 0;
  HCSR04ControlCharacteristic.writeValue(&by, sizeof(long));
  float f = 0;
  HCSR04Characteristic.writeValue(&f, sizeof(float));
#endif

#if COMPILEFOR == LASER
  LaserDistanceService.addCharacteristic(LaserDistanceCharacteristic);
  LaserControlService.addCharacteristic(LaserControlCharacteristic);
  BLE.addService(LaserDistanceService);
  BLE.addService(LaserControlService);
  LaserDistanceCharacteristic.setEventHandler(BLESubscribed, subscribe_occured);
  LaserDistanceCharacteristic.setEventHandler(BLEUnsubscribed, unsubscribe_occured);
  LaserControlCharacteristic.setEventHandler(BLEWritten, write_from_central_occured);
  LaserControlCharacteristic.setEventHandler(BLERead, read_from_central_occured);
  LaserControlCharacteristic.writeValue(&laserMeasuringInterval, sizeof(byte));
#endif
  BLE.setAdvertisedServiceUuid(UUID_MEASURING_TYPE);


  // start advertising
  int result = BLE.advertise();
  Serial.print("Can advertise? ");
  Serial.println(result);
  Serial.println("Smart Drinking Cup BLE Peripheral, waiting for connections....");
  Serial.println(BLE.address());


  imu_accel_data.Ax = 0;
  imu_accel_data.Ay = 0;
  imu_accel_data.Az = 0;
  imu_gyro_data.Gx = 0;
  imu_gyro_data.Gy = 0;
  imu_gyro_data.Gz = 0;
  IMUAccelCharacteristic.writeValue(&imu_accel_data, sizeof(IMU_ACCEL_DATA));
  IMUGyroCharacteristic.writeValue(&imu_gyro_data, sizeof(IMU_GYRO_DATA));


#if COMPILEFOR == CAPACITIVE
  MeasuringTypeCharacteristic.writeValue(CAPACITIVEDEVICE, sizeof(byte) * 14);
#elif COMPILEFOR == LASER
  MeasuringTypeCharacteristic.writeValue(LASERDEVICE, sizeof(byte) * 14);
#elif COMPILEFOR == ULTRASONIC
  MeasuringTypeCharacteristic.writeValue(ULTRASONICDEVICE, sizeof(byte) * 14);
#endif
}

float vectorlength(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

float angleToRestPosition(float x, float y, float z) {
  return acos((x * 0.0 + y * 0.0 + z * 1.0) / (vectorlength(x, y, z) * 1.0)) * (180.0 / PI);
}

bool resting(float x, float y, float z) {
  float accelerationSize = vectorlength(x, y, z);
  if (accelerationSize < 1.04 && accelerationSize > 0.96) {
    float angle = angleToRestPosition(x, y, z);
#if DEBUG && SMART_CUP_POSITION_STATE_DEBUG
    Serial.println("Not Moving");
    Serial.print("Angle ");
    Serial.println(angle);
#endif
    if (abs(angle) < 3) {
#if DEBUG && SMART_CUP_POSITION_STATE_DEBUG
      Serial.println("In Waage");
#endif
      measurablePosition = true;
      return true;
    }
  } else {
#if DEBUG && SMART_CUP_POSITION_STATE_DEBUG
    Serial.println("Moving");
#endif
    measurablePosition = false;
    return false;
  }
  measurablePosition = false;
  return false;
}

bool restingMean() {
  float x = 0;
  float y = 0;
  float z = 0;
  for (int i = 0; i < IMU_VALUE_BUFFER * 3; i += 3) {
    x += meanACCEL[i + 0];
    y += meanACCEL[i + 1];
    z += meanACCEL[i + 2];
  }
  x /= IMU_VALUE_BUFFER * 1.0;
  y /= IMU_VALUE_BUFFER * 1.0;
  z /= IMU_VALUE_BUFFER * 1.0;
  return resting(x, y, z);
}

void registerDrinkEvent() {

  currentDrinkEvent++;
  currentDrinkEvent %= NUMBER_OF_DRINK_EVENTS;

}

#if COMPILEFOR == CAPACITIVE
void measureCapacitive() {
  float mValues = 0;
  for (int i = 0; i < SAMPLES_BEFORE_INC; i++) {
    mValues += analogRead(MEASURE_PIN);
    delay(DELAY_BETWEEN_SAMPLES);
  }
  mValues = ((mValues * 1.0) / SAMPLES_BEFORE_INC);
  measureUpdate.frequency = ad->get_start_frequency() + ad->get_delta_frequency() * sweepIncrements++;
  measureUpdate.response = mValues;
#if DEBUG && AD5932_DEBUG
  Serial.print("MeasurUpdate: freq: ");
  Serial.print(measureUpdate.frequency);
  Serial.print(" response: ");
  Serial.println(measureUpdate.response);
#endif
  //AD5932MeasureResultCharacteristic.writeValue(&measureUpdate, sizeof(AD5932_MEASURE_UPDATE));
  ad->trigger();
}
#endif

void loop() {
  BLE.poll();
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    meanACCEL[imuIndex + 0] = ax;
    meanACCEL[imuIndex + 1] = ay;
    meanACCEL[imuIndex + 2] = az;
#if DEBUG && IMU_DEBUG && IMU_ACCEL_DEBUG
    Serial.println("Accelerometer data: ");
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.println(az);
    Serial.println();
#endif
  }
  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);
    meanGYRO[imuIndex + 0] = gx;
    meanGYRO[imuIndex + 1] = gy;
    meanGYRO[imuIndex + 2] = gz;
#if DEBUG && IMU_DEBUG && IMU_GYRO_DEBUG
    Serial.println("Gyroscope data: ");
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.println(gz);
    Serial.println();
#endif
  }

  if (millis() > (dataupdate + IMU_BLE_UPDATE_INTERVAL)) {
    dataupdate = millis();
    imu_accel_data.Ax = meanACCEL[imuIndex + 0];
    imu_accel_data.Ay = meanACCEL[imuIndex + 1];
    imu_accel_data.Az = meanACCEL[imuIndex + 2];
    IMUAccelCharacteristic.writeValue(&imu_accel_data, sizeof(IMU_ACCEL_DATA));
    imu_gyro_data.Gx = meanGYRO[imuIndex + 0];
    imu_gyro_data.Gy = meanGYRO[imuIndex + 1];
    imu_gyro_data.Gz = meanGYRO[imuIndex + 2];
    IMUGyroCharacteristic.writeValue(&imu_gyro_data, sizeof(IMU_GYRO_DATA));
#if DEBUG && IMU_DEBUG
    Serial.println("IMU BLE Update");
#if USEWIFI
    Serial.println("Time: ");
    if (timeStatus() != timeNotSet) {
      digitalClockDisplay();
    }
#endif
#endif
  }
  imuIndex += 3;
  imuIndex %= (IMU_VALUE_BUFFER * 3);
#if COMPILEFOR == ULTRASONIC
  restingMean();
#endif

#if COMPILEFOR == CAPACITIVE
  if (write_data_to_device) {
    write_data_to_device = false;
    ad->update_settings();
  }
  if (sweepIncrements < ad->get_number_of_increments()) {
    measureCapacitive();
  }
#endif

#if COMPILEFOR == LASER
  if (millis() > (laser_update + laserMeasuringInterval)) {
    laser_update = millis();
    float distance = analogRead(LASER_PIN);
    #if DEBUG && LASER_DEBUG
    float d = ((distance) * (3.3) / (1024)) * 100;
    Serial.print("AnalogRead:");
    Serial.print(distance, 10);
    Serial.print("\tVoltage:");
    Serial.println(d, 10);
    byte values[8];
    long l = *(long*) &distance;
    values[0] = l & 0xFF;
    values[1] = (l >> 8) & 0xFF;
    values[2] = (l >> 16) & 0xFF;
    values[3] = l >> 24;
    l = *(long*) &d;
    values[4] = l & 0xFF;
    values[5] = (l >> 8) & 0xFF;
    values[6] = (l >> 16) & 0xFF;
    values[7] = l >> 24;
    LaserDistanceCharacteristic.writeValue(&values, sizeof(double));
    #endif
    /*
    if (mean_laser_added < 10) {
      mean_laser_distance[mean_laser_added++] = distance;
    } else {
      mean_laser_added = 0;
      distance = 0;
      for (int i = 0; i < 10; i++) {
        distance += mean_laser_distance[i];
      }
      distance /= 10.0f;
      byte values[4];
      long l = *(long*) &distance;
      values[0] = l & 0xFF;
      values[1] = (l >> 8) & 0xFF;
      values[2] = (l >> 16) & 0xFF;
      values[3] = l >> 24;
      LaserDistanceCharacteristic.writeValue(&values, sizeof(double));
#if DEBUG && LASER_DEBUG
      //Serial.print("Laser ");
      //Serial.print("Distance ");
      //Serial.print(distance);
      //Serial.println("cm");
#endif
    }
*/

  }
#endif


#if  COMPILEFOR == ULTRASONIC
  if (!pouring /*&& measurablePosition*/ && millis() > (ultrasonic_update + measuringInterval)) {
    ultrasonic_update = millis();
    hcsr->measure();
    float distance = hcsr->get_distance();
    if (mean_added < 10) {
      mean_distance[mean_added++] = distance;
    } else {
      mean_added = 0;
      distance = 0;
      for (int i = 0; i < 10; i++) {
        distance += mean_distance[i];
      }
      distance /= 10.0f;
      byte values[4];
      long l = *(long*) &distance;
      values[0] = l & 0xFF;
      values[1] = (l >> 8) & 0xFF;
      values[2] = (l >> 16) & 0xFF;
      values[3] = l >> 24;
      HCSR04Characteristic.writeValue(&values, sizeof(float));
#if DEBUG && ULTRASONIC_DEBUG
      Serial.print("Duration ");
      Serial.println(hcsr->get_duration());
      Serial.print("Distance ");
      Serial.print(distance);
      Serial.println("cm");
#endif
    }
  }
  if (!assembly && !pouring && measurablePosition) {
    servoAngle = min_angle;
    //ultrasonic_servo.write(min_angle);
  } else if (!pouring) {
    servoAngle = max_angle;
    //ultrasonic_servo.write(max_angle);
  }
  servoSmooth = (servoAngle *  (1 - prevPercent)) + (servoPrev * prevPercent);
  servoPrev = servoSmooth;
  ultrasonic_servo.write(servoSmooth);
  if (digitalRead(pouringBtnPin) == HIGH && pouringBtnState == LOW) {
    delay(20);
    pouringBtnState = HIGH;
    pouring = true;
    pouring_event = millis();
#if DEBUG
    Serial.println("Pouring");
#endif
    servoAngle = max_angle;
  }
  if (pouring && millis() > pouring_event + maxPouringDuration) {
#if DEBUG
    Serial.println("Pouring End");
#endif
    pouring = false;
    //TODO Move Servo to close position
    servoAngle = min_angle;
    //ultrasonic_servo.write(min_angle);
  }
#endif
  delay(100);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

#if USEWIFI
time_t getNtpTime() {
  ntpc = new ntp(NTPLocalPort);
  IPAddress tumNTP(131, 159, 254, 77);
  //ntpc->updateTimeFromIP(tumNTP);
  ntpc->updateTime();
  unsigned long secsSince1900 = ntpc->getSecsSince19000();
  ntpc->destroy();
  return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
}
#endif
