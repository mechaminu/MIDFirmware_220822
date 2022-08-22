#include <Arduino.h>
#include <SimpleFOC.h>

arduino::MbedI2C _wire(0,1);
arduino::MbedSPI _spi(4,3,2);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(10, 11, 12, 13, 16, 17);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

float xAng, yAng, zAng;
float x, y, z, e;
float _x, _y, _z;

float getAngI2C(int addr) {
  _wire.beginTransmission(addr);
  _wire.write(0x0E);
  _wire.endTransmission();

  _wire.requestFrom(addr, 2);
  while(_wire.available() < 2);

  int resBytes = (_wire.read() << 8) | _wire.read();
  return (float)resBytes / 4096 * 360;
}

void setup() {
  _wire.begin();
  sensor.init(&_spi);
  motor.linkSensor(&sensor);

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 3;
  driver.init();

  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.PID_velocity.P = 0.3f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 1;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 20;
  motor.velocity_limit = 50;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();
  motor.target = 0;

  _delay(1000);

  x = getAngI2C(0x40);
  y = getAngI2C(0x41);
  z = getAngI2C(0x42);
  e = sensor.getAngle();

  xAng = 0;
  yAng = 0;
  zAng = 0;
}

void loop() {
  float elbowAng = -(sensor.getAngle()-e)/PI*180/5;
  _x = getAngI2C(0x40);
  _y = getAngI2C(0x41);
  _z = getAngI2C(0x42);

  xAng += x - _x;
  yAng -= y - _y;
  zAng += z - _z;

  x = _x; 
  y = _y;
  z = _z;

  byte buf[2+4*4];
  buf[0] = 0x02;
  memcpy(&buf[1+4*(1-1)],&zAng, 4);
  memcpy(&buf[1+4*(2-1)],&yAng, 4);
  memcpy(&buf[1+4*(3-1)],&xAng, 4);
  memcpy(&buf[1+4*(4-1)],&elbowAng, 4);
  buf[2+4*4-1] = 0x04;

  Serial.write(buf,18);

  motor.loopFOC();
  motor.move();
}