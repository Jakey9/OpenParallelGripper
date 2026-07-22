// ============================================================================
// OpenParallelGripper — Zhonglin Servo + Modbus RTU Bridge (ESP32 WROOM)
// ============================================================================
//
// Uncomment the line below to enter TUNING MODE.
// In tuning mode, Serial (USB) is used for debug output instead of Modbus.
// Disconnect the RS485 module when using tuning mode.
//
#define TUNING_MODE

#ifndef TUNING_MODE
#include <ModbusRTU.h>
#endif

// ========================== TUNABLE PARAMETERS ==============================

#define SERVO_ID         0        // Zhonglin servo ID (factory default is usually 0 or 1)
#define SERVO_BAUDRATE   115200

// Gripper open/close positions in PWM units (500–2500, maps to 0–270 deg).
// Use TUNING_MODE to discover the correct values for your setup.
#define GRIPPER_OPEN_POS   830
#define GRIPPER_CLOSE_POS  1900

#define SERVO_MOVE_TIME  500      // milliseconds for each move command
#define SERVO_CMD_DELAY  10       // ms delay after sending a command

// ========================== PIN DEFINITIONS =================================

// Zhonglin servo via FE-URT module (UART2 remapped)
#define TX_SERVO  27   // ESP32 GPIO27 -> FE-URT TX
#define RX_SERVO  14   // ESP32 GPIO14 -> FE-URT RX

// Modbus RTU via RS485-to-TTL module (UART0 — shares pins with USB bridge)
// TX0 = GPIO1, RX0 = GPIO3 (default Serial pins)
// NOTE: disconnect RS485 module when uploading or using TUNING_MODE.

// Digital inputs from Lite6 tool outputs
#define LITE6_OUTPUT0  21
#define LITE6_OUTPUT1  19

// ========================== MODBUS REGISTERS ================================

#define SLAVE_ID              1
#define REG_GRIPPER_POS       128   // Holding register: target position (PWM)
#define REG_READ_GRIPPER_POS  257   // Input register: current position (PWM)

// ========================== GLOBALS =========================================

int lite6_0;
int lite6_1;
uint16_t gripper_pos    = 0;
uint16_t current_pos    = 0;
bool     torque_active  = false;

#ifndef TUNING_MODE
ModbusRTU mb;
#endif

// ========================== ZHONGLIN ASCII PROTOCOL =========================

String zhonglin_send(const String& cmd) {
  Serial2.print(cmd);
  delay(SERVO_CMD_DELAY);
  String resp = "";
  while (Serial2.available()) {
    resp += (char)Serial2.read();
  }
  return resp;
}

void zhonglin_unlock(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPULK!", id);
  zhonglin_send(buf);
}

void zhonglin_restore(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPULR!", id);
  zhonglin_send(buf);
}

void zhonglin_move(int id, int pwm, int time_ms) {
  char buf[32];
  snprintf(buf, sizeof(buf), "#%03dP%04dT%04d!", id, pwm, time_ms);
  zhonglin_send(buf);
}

int zhonglin_read_pos(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPRAD!", id);
  String resp = zhonglin_send(buf);

  // Response format: #001P1500!  — extract the 4-digit PWM value after 'P'
  int idx = resp.indexOf('P', 1);  // skip leading '#'
  if (idx < 0) return -1;
  String pwm_str = resp.substring(idx + 1, idx + 5);
  return pwm_str.toInt();
}

void zhonglin_stop(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPDST!", id);
  zhonglin_send(buf);
}

void zhonglin_set_poweron_unlock(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPCSM!", id);
  zhonglin_send(buf);
}

void zhonglin_set_poweron_restore(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPCSR!", id);
  zhonglin_send(buf);
}

void zhonglin_set_min(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPSMI!", id);
  zhonglin_send(buf);
}

void zhonglin_set_max(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPSMX!", id);
  zhonglin_send(buf);
}

String zhonglin_read_temp_voltage(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPRTV!", id);
  return zhonglin_send(buf);
}

String zhonglin_read_version(int id) {
  char buf[16];
  snprintf(buf, sizeof(buf), "#%03dPVER!", id);
  return zhonglin_send(buf);
}

float pwm_to_degrees(int pwm) {
  return (float)(pwm - 500) / 2000.0f * 270.0f;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  pinMode(LITE6_OUTPUT0, INPUT_PULLUP);
  pinMode(LITE6_OUTPUT1, INPUT_PULLUP);

  // Zhonglin servo UART (FE-URT module)
  Serial2.begin(SERVO_BAUDRATE, SERIAL_8N1, RX_SERVO, TX_SERVO);
  delay(100);

  // Safety: release torque immediately so servo does not move on power-up
  zhonglin_unlock(SERVO_ID);
  delay(50);

  // Read initial position
  current_pos = zhonglin_read_pos(SERVO_ID);

#ifdef TUNING_MODE
  // USB serial for debug output (RS485 must be disconnected)
  Serial.begin(115200);
  delay(500);

  Serial.println("==============================================");
  Serial.println("  Zhonglin Servo TUNING MODE");
  Serial.println("==============================================");
  Serial.print("Servo ID: "); Serial.println(SERVO_ID);

  String ver = zhonglin_read_version(SERVO_ID);
  Serial.print("Firmware : "); Serial.println(ver);

  String tv = zhonglin_read_temp_voltage(SERVO_ID);
  Serial.print("Temp/Volt: "); Serial.println(tv);

  if (current_pos >= 0) {
    Serial.print("Position : PWM="); Serial.print(current_pos);
    Serial.print(" | Angle="); Serial.print(pwm_to_degrees(current_pos), 1);
    Serial.println(" deg");
  } else {
    Serial.println("WARNING: could not read servo position!");
  }

  Serial.println();
  Serial.println("Commands (send via Serial Monitor):");
  Serial.println("  l  - release torque (free to move by hand)");
  Serial.println("  r  - restore torque (servo holds position)");
  Serial.println("  u  - set power-on mode to UNLOCK (persistent)");
  Serial.println("  k  - set power-on mode to LOCK (persistent)");
  Serial.println("  m  - save current pos as MIN limit (persistent)");
  Serial.println("  x  - save current pos as MAX limit (persistent)");
  Serial.println("  c  - move to center (PWM 1378)");
  Serial.println("  o  - move to GRIPPER_OPEN_POS");
  Serial.println("  p  - move to GRIPPER_CLOSE_POS");
  Serial.println("  t  - read temperature and voltage");
  Serial.println("  f  - factory reset (except ID)");
  Serial.println("==============================================");
  Serial.println();

#else
  // Normal mode: Modbus RTU on Serial (UART0, TX0/RX0 = GPIO1/3)
  Serial.begin(115200, SERIAL_8N1);
  mb.begin(&Serial);
  mb.slave(SLAVE_ID);

  mb.addHreg(REG_GRIPPER_POS);
  mb.Hreg(REG_GRIPPER_POS, (current_pos > 0) ? current_pos : 1500);
  mb.addIreg(REG_READ_GRIPPER_POS, (current_pos > 0) ? current_pos : 0);

  // Restore torque so servo is ready for commands
  zhonglin_restore(SERVO_ID);
  torque_active = true;
#endif

  delay(500);
}

// ============================================================================
// LOOP
// ============================================================================

#ifdef TUNING_MODE

void loop() {
  // Continuously read and display position
  int pos = zhonglin_read_pos(SERVO_ID);
  if (pos >= 0) {
    current_pos = pos;
    Serial.print("PWM: "); Serial.print(pos);
    Serial.print(" | Angle: "); Serial.print(pwm_to_degrees(pos), 1);
    Serial.println(" deg");
  }

  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    // Flush remaining chars in the line
    while (Serial.available()) Serial.read();

    switch (cmd) {
      case 'l':
        zhonglin_unlock(SERVO_ID);
        torque_active = false;
        Serial.println(">> Torque RELEASED — move servo by hand");
        break;
      case 'r':
        zhonglin_restore(SERVO_ID);
        torque_active = true;
        Serial.println(">> Torque RESTORED — servo holds position");
        break;
      case 'u':
        zhonglin_set_poweron_unlock(SERVO_ID);
        Serial.println(">> Power-on mode set to UNLOCK (saved to flash)");
        break;
      case 'k':
        zhonglin_set_poweron_restore(SERVO_ID);
        Serial.println(">> Power-on mode set to LOCK (saved to flash)");
        break;
      case 'm':
        zhonglin_set_min(SERVO_ID);
        Serial.print(">> Current position saved as MIN limit (PWM=");
        Serial.print(current_pos); Serial.println(")");
        break;
      case 'x':
        zhonglin_set_max(SERVO_ID);
        Serial.print(">> Current position saved as MAX limit (PWM=");
        Serial.print(current_pos); Serial.println(")");
        break;
      case 'c':
        zhonglin_restore(SERVO_ID);
        torque_active = true;
        zhonglin_move(SERVO_ID, 1378, 1000);
        Serial.println(">> Moving to center (PWM=1378)");
        break;
      case 'o':
        zhonglin_restore(SERVO_ID);
        torque_active = true;
        zhonglin_move(SERVO_ID, GRIPPER_OPEN_POS, SERVO_MOVE_TIME);
        Serial.print(">> Moving to OPEN position (PWM=");
        Serial.print(GRIPPER_OPEN_POS); Serial.println(")");
        break;
      case 'p':
        zhonglin_restore(SERVO_ID);
        torque_active = true;
        zhonglin_move(SERVO_ID, GRIPPER_CLOSE_POS, SERVO_MOVE_TIME);
        Serial.print(">> Moving to CLOSE position (PWM=");
        Serial.print(GRIPPER_CLOSE_POS); Serial.println(")");
        break;
      case 't': {
        String tv = zhonglin_read_temp_voltage(SERVO_ID);
        Serial.print(">> Temp/Voltage: "); Serial.println(tv);
        break;
      }
      case 'f':
        Serial.println(">> Factory reset (except ID)...");
        char fbuf[16];
        snprintf(fbuf, sizeof(fbuf), "#%03dPCLE!", SERVO_ID);
        zhonglin_send(fbuf);
        Serial.println(">> Done. Power-cycle the servo.");
        break;
      default:
        break;
    }
  }

  delay(200);
}

#else  // Normal operation mode

void loop() {
  lite6_0 = digitalRead(LITE6_OUTPUT0);
  lite6_1 = digitalRead(LITE6_OUTPUT1);

  // Digital input: open
  if (lite6_0 == LOW && lite6_1 == HIGH) {
    zhonglin_move(SERVO_ID, GRIPPER_OPEN_POS, SERVO_MOVE_TIME);
  }
  // Digital input: close
  else if (lite6_0 == HIGH && lite6_1 == LOW) {
    zhonglin_move(SERVO_ID, GRIPPER_CLOSE_POS, SERVO_MOVE_TIME);
  }
  // Modbus control mode (both HIGH)
  else if (lite6_0 == HIGH && lite6_1 == HIGH) {
    gripper_pos = mb.Hreg(REG_GRIPPER_POS);
    zhonglin_move(SERVO_ID, gripper_pos, SERVO_MOVE_TIME);
  }

  // Read current position and update Modbus input register
  int pos = zhonglin_read_pos(SERVO_ID);
  if (pos >= 0) {
    current_pos = pos;
    mb.Ireg(REG_READ_GRIPPER_POS, current_pos);
  }

  mb.task();
  yield();
}

#endif
