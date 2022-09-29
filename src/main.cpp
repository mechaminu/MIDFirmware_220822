#include "mbed.h"
#include <Arduino.h>
#include <SimpleFOC.h>

MbedSPI spi(4,3,2);
MbedI2C wire(16,17);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(8, 11, 9, 12, 10, 13);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

float xAng, yAng, zAng, elbowAng;
float _x, _y, _z, _e;
unsigned long prevTime;

int getAngI2C(int addr, float& ang) {
  wire.requestFrom(addr, 6);
  delayMicroseconds(20);
  while (wire.available() > 0) {
    if (wire.read() == 0x02) {
      wire.readBytesUntil(0x04,(uint8_t*)&ang,4);
      return 1;
    }
  }
  return 0;
}

void getAng(int addr, float &target, float offset, float ratio) {
  float res;
  if (getAngI2C(addr,res))
    target = (res - offset) / PI * 180 / ratio;
}

byte buf_tq[16];
float xTorque, eTorque;
void torqueCommand() {
  while (Serial.available() > 0) {
    if (Serial.read() == 0x02) {
      switch (Serial.read()) {
        case 0xFF:
          Serial.readBytesUntil(0x04, buf_tq, 16);
          memcpy(&xTorque,&buf_tq[4*2],4);
          memcpy(&eTorque,&buf_tq[4*3],4);
      } 
    }
  }
  motor.target = eTorque / 1.33;

  uint8_t buf[6];
  buf[0] = 0x02;
  memcpy(&buf[1],(uint8_t*)&xTorque,4);
  buf[5] = 0x04;

  wire.beginTransmission(0x01);
  wire.write(buf,6);
  Serial.println(wire.endTransmission());
}

void setup() {

  for (int i = 0; i < 5; i++) {
    digitalWrite(17,HIGH);
    digitalWrite(16,HIGH);
    delay(500);
    digitalWrite(17,LOW);
    digitalWrite(16,LOW);
    delay(500);
  }

  wire.begin();

  Serial.begin(115200);
  // motor.useMonitoring(Serial);

  sensor.init(&spi);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  // motor.initFOC();
  motor.initFOC(5.78, Direction::CCW);
  motor.target = 0;

  // Initial value for zeroing
  getAngI2C(0x01, _x);
  _e = sensor.getAngle();
  _y = 0;
  _z = 0;

  // getAngI2C(0x02, _y);
  // getAngI2C(0x03, _z);
}

byte buf[2+4*4];
void loop() {
  
  motor.loopFOC();
  motor.move();

  getAng(0x01, xAng, _x, -40);
  elbowAng = -(sensor.getAngle() - _e)/PI*180/5;
  // getAng(0x02, yAng, _y, 1);
  // getAng(0x03, zAng, _z, -40);

  Serial.print(elbowAng);
  Serial.print('\t');
  Serial.print(xAng);
  Serial.print('\n');

  unsigned long curTime = millis();
  if (curTime - prevTime > 20) {
    buf[0] = 0x02;
    memcpy(&buf[1+4*(1-1)],&zAng, 4);
    memcpy(&buf[1+4*(2-1)],&yAng, 4);
    memcpy(&buf[1+4*(3-1)],&xAng, 4);
    memcpy(&buf[1+4*(4-1)],&elbowAng, 4);
    buf[2+4*4-1] = 0x04;
    // Serial.write(buf,18);
    torqueCommand();

    prevTime = curTime;
  }
}