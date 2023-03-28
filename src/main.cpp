#include <Arduino.h>
#include <stdio.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

//6 wire BLDC drive with TMC6300
BLDCDriver6PWM driver = BLDCDriver6PWM(0,2,4,6,8,10);
//low side sensing with INA180A1 20V/V current amplifier with 150mOhm shunt
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.15, 20, 26, 27);
//aliexpress motor with 11? pole pair, 5ohm phase resistance and supposed 190KV rating
BLDCMotor motor = BLDCMotor(1,1,4000);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }

void setup() {
  adc_init();
  adc_gpio_init(28);
  adc_select_input(2); //gpio28
  // put your setup code here, to run once:
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit = 5;
  driver.init();

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.current_limit = 0.5;
  motor.init();
  motor.initFOC();

  //current_sense.linkDriver(&driver);
  //current_sense.init();

  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  motor.target = 0;
  _delay(1000);

}

void loop() {
  uint16_t targetCounter = 0;
  motor.loopFOC();
  // open loop velocity movement
  // using motor.current_limit and motor.velocity_limit
  motor.move();
  // user communication
  command.run();
  //uint16_t potRead = map(adc_read(),660,4096,0,100);
  //motor.target = -20;
  //Serial.println(adc_read());

}