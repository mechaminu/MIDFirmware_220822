#include <Arduino.h>
#include <SimpleFOC.h>

MbedSPI _spi(4, 3, 2);
// MbedI2C _wire(26, 27);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(10, 11, 12, 13, 16, 17);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

byte buf[2 + 4 * 4];
float xAng, yAng, zAng, elbowAng;
float x, y, z, e;
float _x, _y, _z;
float prevTime, curTime;

float getAngI2C(int addr)
{
  // _wire.beginTransmission(addr);
  // _wire.write(0x0E);
  // _wire.endTransmission();
  // _wire.requestFrom(addr, 2);
  // char buf[2];
  // int resBytes = _wire.readBytes(buf,2) ? 0 : ((buf[0] << 8) | buf[1]);
  // return (float)resBytes / 4096 * 360;
  return 0;
}

float res;
unsigned long lastCommand;
void doSerial()
{
  while (Serial.available() > 0)
  {
    if (Serial.read() == 0x02)
    {
      uint8_t getB = Serial.read();
      if (getB == 0xFF)
      {
        Serial.readBytesUntil(0x04, (byte *)&res, 4);
        motor.target = res / 1.333;
        lastCommand = millis();
      }
      else if (getB == 0xAA)
      {
        buf[0] = 0x02;
        memcpy(&buf[1 + 4 * (1 - 1)], &zAng, 4);
        memcpy(&buf[1 + 4 * (2 - 1)], &yAng, 4);
        memcpy(&buf[1 + 4 * (3 - 1)], &xAng, 4);
        memcpy(&buf[1 + 4 * (4 - 1)], &elbowAng, 4);
        buf[2 + 4 * 4 - 1] = 0x04;
        Serial.write(buf, 18);
      }
    }
  }
}

void setup()
{
  // _wire.begin();
  Serial.begin(115200);
  sensor.init(&_spi);
  motor.linkSensor(&sensor);

  driver.init();
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4;

  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.PID_velocity.P = 0.3f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 20;
  motor.voltage_sensor_align = 1;
  motor.velocity_limit = 50;

  motor.init();
  motor.initFOC(6.25, Direction::CCW);
  motor.sensor_offset = -4.415;
  motor.target = 0;

  x = getAngI2C(0x40);
  y = getAngI2C(0x41);
  z = getAngI2C(0x42);
  e = sensor.getAngle();

  xAng = 0;
  yAng = 0;
  zAng = 0;

  prevTime = millis();
  lastCommand = millis();
}

void getAngles() {
  _x = getAngI2C(0x40);
  _y = getAngI2C(0x41);
  _z = getAngI2C(0x42);
  xAng += x - _x;
  yAng -= y - _y;
  zAng += z - _z;
  elbowAng = -(sensor.getAngle() - e) / PI * 180 / 5;
  x = _x;
  y = _y;
  z = _z;
}

void loop()
{
  motor.loopFOC();
  motor.move();

  getAngles();
  doSerial();

  if (millis() - lastCommand > 1000)
    motor.target = 0;
}