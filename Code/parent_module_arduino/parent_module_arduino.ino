#include <ESP32Servo.h>
#include <esp_now.h>
#include "WiFi.h"
#include "mav1/common/mavlink.h"
#include <WiFiClient.h>
#include <HardwareSerial.h>

#include <DynamixelShield.h>

#define SERIAL_BAUDRATE 921600


const char *ssid = "ESP32_AP";      // Whatever you want the wifi name to appear as
const char *password = "password";  // Your wifi password to the esp32 master module
WiFiServer server(80);

int moduleID = 3;

const int motor1 = 1;  // Opposite Compliance Pad
const int motor2 = 2;  // Clockwise from motor1
const int motor3 = 3;  // Anti-clockwise from motor1
const int winchMotor = 33;
const int propellerMotor = 32;
unsigned int yawMin = 977;     // Minimum yaw pulse width
unsigned int yawMax = 1990;    // Maximum yaw pulse width
unsigned int rollMin = 977;    // Minimum roll pulse width
unsigned int rollMax = 1990;   // Maximum roll pulse width
unsigned int pitchMin = 977;   // Minimum pitch pulse width
unsigned int pitchMax = 1990;  // Maximum pitch pulse width
unsigned int armThreshold = 1500;

int mappedYaw = 0;
int mappedRoll = 0;
int mappedPitch = 0;
bool armed = false;
int rosWinch = 0;

Servo winch;
Servo propeller;

float omega = 0;
float vx = 0;
float vy = 0;


struct data_package {
  int8_t yaw;
  int8_t roll;
  int8_t pitch;
  int8_t arm;
  int8_t moduleSelect = 0;
  int8_t winchSingle = 0;
  int8_t winchAll = 0;
  uint16_t servo_output1 = 900;
  uint16_t servo_output2 = 900;
  uint16_t servo_output3 = 900;
  uint16_t servo_output4 = 900;
};
data_package data;


// Control loop and signal sending timer
long send_signal_timer = 0;
long control_loop_timer = 0;
long actuator_timer = 0;
int state = 1;

// Mavlink Code
// ----------------------------------------------------------------------------------------------------------------------//
// Variables for mavlink
const int rate = 5000;
const int system_id = 255;
const int component_id = 0;
uint8_t msg_buf[256];
int chan = MAVLINK_COMM_0;
mavlink_status_t status;

// Parse incoming MAVLink messages
mavlink_message_t msg;
mavlink_servo_output_raw_t servo_msg;
mavlink_rc_channels_t rc_channel_msg;

void request_rc_channels() {
  mavlink_message_t msg_r;

  // Request RC channels at 100Hz
  mavlink_msg_command_long_pack(system_id, component_id, &msg_r, 1, 0, 511, 0, 65, rate, 0, 0, 0, 0, 0);

  int msg_len = mavlink_msg_to_send_buffer(msg_buf, &msg_r);

  // change to serial 2
  Serial2.write(msg_buf, msg_len);
}

void request_servo() {
  mavlink_message_t msg_r;
  // Request Servo output at 100Hz
  mavlink_msg_command_long_pack(system_id, component_id, &msg_r, 1, 0, 511, 0, 36, rate, 0, 0, 0, 0, 0);

  int msg_len = mavlink_msg_to_send_buffer(msg_buf, &msg_r);

  // change to serial 2
  Serial2.write(msg_buf, msg_len);
}

// ----------------------------------------------------------------------------------------------------------------------//


uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_info_t peerInfo;



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // char
}

WiFiClient client = server.available();
const float DXL_PROTOCOL_VERSION = 2.0;
// DynamixelShield dxl;

Dynamixel2Arduino dxl(Serial, 27);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void dynamixelSetup() {
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  delay(3000);
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
void espWifiSetup(){
  // Device is set up as an access point and station
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  server.begin();
  // For ROS to connect to
  // Serial.println("Server started");
  // Serial.print("AP IP address: ");
  // Serial.println(ip);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  if (esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_36M) != ESP_OK) {
    // Serial.println("Failed to set new rate");
  };

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    // Serial.println("Failed to add peer1");
    return;
  }
}

void setup() {
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, 16, 17);
  dynamixelSetup();
  delay(1000);

  espWifiSetup();


  winch.attach(winchMotor);
  propeller.attach(propellerMotor);

  // Request the mavlink messages needed, in this case just the rc channels and the servo outputs for propeller
  request_rc_channels();
  request_servo();
  delay(1000);
}

void loop() {
  // Check for available data on the serial port
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle the parsed message
      switch (msg.msgid) {
        // Add more cases for other message types here
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
          {
            mavlink_msg_servo_output_raw_decode(&msg, &servo_msg);
            propeller.writeMicroseconds(servo_msg.servo3_raw);
            controlSignal(rc_channel_msg, servo_msg);
            break;
          }
        case MAVLINK_MSG_ID_RC_CHANNELS:
          {
            mavlink_msg_rc_channels_decode(&msg, &rc_channel_msg);
            if ((rc_channel_msg.chan7_raw > 1700) & (rc_channel_msg.chan6_raw > 1700)) {
              state = 2;
            }
            break;
          }
        default:
          // Handle unknown message types or ignore them
          // Serial.println(msg.msgid);
          break;
      }
    }
    if (millis() - send_signal_timer >= 10) {
      // A simple FSM where at the default state is listening to ROS for autonomous operations, once the module select triggers are both pulled down manual control kicks in 
      switch (state) {
        case 1:
          wifi_data(rc_channel_msg); // Listening to ROS for autonomous operation commands
          break;
        case 2:
          controlSignal(rc_channel_msg, servo_msg); // Listening to the RC messages
          break;
      }
      send_signal_timer = millis();
    }
    if (millis() - control_loop_timer >= 20) {
      if (armed) {
        Stop();
      } else {
        executeCommand();
      }
      control_loop_timer = millis();
    }
  }
}

// This function handles the ros messages sent to this module
void wifi_data(mavlink_rc_channels_t &rc_channel_msg) {
  if (!client) {
    client = server.available();
  }
  if (client.connected()) {
    String wifi_data = client.readStringUntil('\n');
    int id, winch;
    float vx_wifi, vy_wifi, wz;
    const char *wifi_data_char = wifi_data.c_str();
    sscanf(wifi_data_char, "%d,%f,%f,%f,%d", &id, &vx_wifi, &vy_wifi, &wz, &winch) == 5;
    data.moduleSelect = id;
    data.pitch = (int)(vx_wifi);
    data.roll = (int)(vy_wifi);
    data.yaw = (int)(wz);
    data.winchSingle = winch;
    if(id == moduleID){
      rosWinch = winch;
      omega = map(wz, -100, 100, -720, 720) / 1000.0;  //remapped to rad/s for velocity control
      vx = map(vy_wifi, -100, 100, -72, 72) / 1000.0;      //remapped to m/s for velocity control
      vy = map(vx_wifi, -100, 100, -125, 125) / 1000.0;   //remapped to m/s for velocity control
      
      actuator_timer = millis();
    }else if (millis() - actuator_timer >= 1000) {
      omega = 0;
      vx = 0;
      vy = 0;
      actuator_timer = millis();
    }
    data.winchAll = rc_channel_msg.chan9_raw < 1300 ? -1 : (rc_channel_msg.chan9_raw > 1700 ? 1 : 0);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
  }
}

void Stop() {
  dxl.setGoalVelocity(motor1, 0, UNIT_RPM);
  dxl.setGoalVelocity(motor2, 0, UNIT_RPM);
  dxl.setGoalVelocity(motor3, 0, UNIT_RPM);
  winch.write(90);
}

void executeCommand() {
  if (data.winchAll == 1 || rosWinch == 1) {
    winch.write(180);
  } else if (data.winchAll == -1 || rosWinch == -1) {
    winch.write(0);
  } else if (data.moduleSelect == moduleID) {
    if (data.winchSingle == 1) {
      winch.write(180);
    } else if (data.winchSingle == -1) {
      winch.write(0);
    } else {
      winch.write(90);
    }
  } else {
    winch.write(90);
  }

  float R = 0.20;
  float r = 0.024;
  float pi = 3.1415926;
  float speed1 = (2 * vx - R * omega) / r * 60 / (2 * pi);
  float speed2 = (-vx + sqrt(3) * vy - R * omega) / r * 60 / (2 * pi);
  float speed3 = (-vx - sqrt(3) * vy - R * omega) / r * 60 / (2 * pi);  // Calculating it in rpm

  dxl.setGoalVelocity(motor1, -speed1, UNIT_RPM);
  dxl.setGoalVelocity(motor2, -speed2, UNIT_RPM);
  dxl.setGoalVelocity(motor3, -speed3, UNIT_RPM);
}

void controlSignal(mavlink_rc_channels_t &rc_channel_msg, mavlink_servo_output_raw_t &servo_msg) {
  // Read PWM signals
  unsigned int yawValue = rc_channel_msg.chan1_raw;
  unsigned int rollValue = rc_channel_msg.chan4_raw;
  unsigned int pitchValue = rc_channel_msg.chan3_raw;
  unsigned int armValue = rc_channel_msg.chan5_raw;
  // Map PWM values to desired range
  mappedYaw = map(yawValue, yawMin, yawMax, -100, 100);
  mappedRoll = map(rollValue, rollMin, rollMax, -100, 100);
  mappedPitch = map(pitchValue, pitchMin, pitchMax, -100, 100);
  armed = armValue > armThreshold;
  // Set values to send
  data.yaw = mappedYaw;
  data.roll = mappedRoll;
  data.pitch = mappedPitch;
  data.arm = armed;
  data.winchAll = rc_channel_msg.chan9_raw < 1300 ? -1 : (rc_channel_msg.chan9_raw > 1700 ? 1 : 0);
  data.winchSingle = rc_channel_msg.chan8_raw < 1300 ? -1 : (rc_channel_msg.chan8_raw > 1700 ? 1 : 0);
  data.moduleSelect = ((rc_channel_msg.chan7_raw > 1300) | ((rc_channel_msg.chan6_raw > 1300) << 1)) + 1;
  data.servo_output1 = servo_msg.servo1_raw;
  data.servo_output2 = servo_msg.servo2_raw;
  data.servo_output3 = servo_msg.servo3_raw;
  data.servo_output4 = servo_msg.servo4_raw;
  if(data.moduleSelect == moduleID){
    rosWinch = data.winchSingle;
    omega = map(data.yaw, -100, 100, -720, 720) / 1000.0;  //remapped to rad/s for velocity control
    vx = map(data.roll, -100, 100, -72, 72) / 1000.0;      //remapped to m/s for velocity control
    vy = map(data.pitch, -100, 100, -125, 125) / 1000.0;   //remapped to m/s for velocity control
    
    actuator_timer = millis();
  }else if (millis() - actuator_timer >= 1000) {
    omega = 0;
    vx = 0;
    vy = 0;
    actuator_timer = millis();
  }

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
}
