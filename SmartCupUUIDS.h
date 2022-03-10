#ifndef SmartCupUUIDS_h
#define SmartCupUUIDS_h
#include "Arduino.h"

const inline char* UUID_MEASURING_TYPE = "072585df-390b-434e-ba25-116533274d3e";
const inline char* UUID_AD5932CONFIGURATION = "adef44b0-3308-42f7-9bba-65c648d7bf66";
const inline char* UUID_IMU_ACCEL = "cb68774e-5438-4c5a-8937-814be0a249bb";
const inline char* UUID_GYRO_IMU = "cd98f4b7-df60-4d9c-9c87-6a64e19a7c4f";
const inline char* UUID_AD5932TRIGGER = "cfe2a28b-3e8a-4023-b2b9-70c773a27fb5";
const inline char* UUID_REPLY = "1c54116d-d6aa-4d1f-96a4-5827842c3e4f";
const inline char* UUID_HCSR04_DISTANCE = "6a2676fa-20e0-48a7-9cd6-560a48013bfe";
const inline char* UUID_HCSR04_CONTROL = "9988258d-5663-4e1f-a4bc-a8e211f98e39";
const inline char* UUID_LASER_CONTROL = "849831dc-427b-4784-8848-1d88ef7e4d44";
const inline char* UUID_LASER_DISTANCE = "670b40c4-df34-4f4a-a59d-242c51e7774a";
const inline char* UUID_WIFI_RELATED = "e146d40a-6fdf-4887-86e1-0c4875b349e6";
const inline char* UUID_AD5932_MEASURE_RESULT = "cbd047db-ed1e-4523-bf97-f29f5b4f4c8d";

const byte CAPACITIVEDEVICE[] = {0x43, 0x61, 0x70, 0x61, 0x63, 0x69, 0x74, 0x69, 0x76, 0x65, 0x20, 0x43, 0x75, 0x70};
const byte LASERDEVICE[]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x61, 0x73, 0x65, 0x72, 0x20, 0x43, 0x75, 0x70};
const byte ULTRASONICDEVICE[] = {0x55, 0x6c, 0x74, 0x72, 0x61, 0x73, 0x6f, 0x6e, 0x69, 0x63, 0x20, 0x43, 0x75, 0x70};

#endif
