#include <ESP32Servo.h>
#include <esp_now.h>
#include "WiFi.h"
#include "esp_private/wifi.h"
#include <DynamixelShield.h>

int moduleID = 4; //1,2 or 4

const int motor1 = 1;     // Opposite Compliance Pad
const int motor2 = 2;    // Clockwise from motor1 
const int motor3 = 3;    // Anti-clockwise from motor1 
const int winchMotor  = 33;
const int propellerMotor = 32;

Servo winch;
Servo propeller;

float omega = 0; 
float vx = 0;
float vy = 0; 

// Control loop and signal sending timer
long control_loop_timer = 0;
long actuator_timer = 0;

struct data_package {
  int8_t yaw;
  int8_t roll;
  int8_t pitch;
  int8_t arm;
  int8_t moduleSelect;
  int8_t winchSingle;
  int8_t winchAll;
  uint16_t servo_output1;
  uint16_t servo_output2;
  uint16_t servo_output3;
  uint16_t servo_output4;
};

data_package data; 


const float DXL_PROTOCOL_VERSION = 2.0;
// DynamixelShield dxl;

Dynamixel2Arduino dxl(Serial2, 27);

//This namespace is required to use Control table item names
using namespace ControlTableItem;
int rosWinch = 0;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  propeller.writeMicroseconds(data.servo_output4);
  if (data.moduleSelect == moduleID){
    omega = map(data.yaw, -100, 100, -720, 720)/1000.0; //remapped to rad/s for velocity control 
    vx = map(data.roll, -100, 100, -72, 72)/1000.0; //remapped to m/s for velocity control
    vy = map(data.pitch, -100, 100, -125, 125)/1000.0; //remapped to m/s for velocity control
    actuator_timer = millis();
    rosWinch = data.winchSingle;
  } else if(millis()-actuator_timer >= 1000){ // If module doesn't receive any new signal for more than one second stop motors
    omega = 0; 
    vx = 0; 
    vy = 0;
    rosWinch = 0;
    actuator_timer = millis();
  }

}

void dynamixelSetup() {
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(motor1);
  dxl.ping(motor2);
  dxl.ping(motor3);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(motor1);
  dxl.torqueOff(motor2);
  dxl.torqueOff(motor3);
  dxl.setOperatingMode(motor1, OP_VELOCITY);
  dxl.setOperatingMode(motor2, OP_VELOCITY);
  dxl.setOperatingMode(motor3, OP_VELOCITY);
  dxl.torqueOn(motor1);
  dxl.torqueOn(motor2);
  dxl.torqueOn(motor3);
}

void setup() {
  // Serial.begin(921600);   // Initialize SoftwareSerial with baud rate of 1000000
  dynamixelSetup();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");  // Use SoftSerial here
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  
  data.yaw = 0;
  data.roll = 0;
  data.pitch = 0;
  propeller.attach(propellerMotor);
  winch.attach(winchMotor);
}

void loop() {
  if(millis()-control_loop_timer>= 20){
    if(data.arm){
      Stop();
    }else{
      executeCommand();
    }
    control_loop_timer = millis();
  }
}

void Stop(){
  dxl.setGoalVelocity(motor1, 0, UNIT_RPM);
  dxl.setGoalVelocity(motor2, 0, UNIT_RPM);
  dxl.setGoalVelocity(motor3, 0, UNIT_RPM);
  winch.write(90);
}

void executeCommand(){
  if(data.winchAll == 1 || rosWinch == 1){
    winch.write(180);
  }else if(data.winchAll == -1 || rosWinch == -1){
    winch.write(0);
  }else if(data.moduleSelect == moduleID){
    if(data.winchSingle == 1){
      winch.write(180);
    }else if(data.winchSingle == -1){
      winch.write(0);
    }else{
      winch.write(90);
    }
  }else{
    winch.write(90);
  }

  float R = 0.20;
  float r = 0.024;
  float pi = 3.1415926;
  float speed1 = (2*vx - R * omega) / r * 60 / (2*pi);
  float speed2 = (-vx + sqrt(3) * vy - R * omega) / r * 60 / (2*pi);
  float speed3 = (-vx - sqrt(3) * vy - R * omega) / r * 60 / (2*pi); // Calculating it in rpm

  dxl.setGoalVelocity(motor1, -speed1, UNIT_RPM);
  dxl.setGoalVelocity(motor2, -speed2, UNIT_RPM);
  dxl.setGoalVelocity(motor3, -speed3, UNIT_RPM);
}