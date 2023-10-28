/**
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <SimpleFOC.h>
#include <QuickPID.h>

// Motor instance
BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
// Encoder encoder = Encoder(A_HALL2, A_HALL3, 2048, A_HALL1);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);

// Interrupt routine intialisation
// channel A and B callbacks

float starting_pos, current_pos, output;

float Kp = 0.25, Ki = 0, Kd = 0;
float Kp2 = 0.05, Ki2 = 0, Kd2 = 0;

long loop_count = 0;
long loop_time = 0;
long loop_rate = 0;
// Specify PID links
QuickPID myPID(&current_pos, &output, &starting_pos);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.motion(&motor, cmd); }

void setup()
{

  // initialize encoder sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 80;
  ;
  motor.current_limit = 2;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
  sensor.update();
  starting_pos = 0;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(myPID.Control::automatic);
  myPID.SetOutputLimits(0.3, 10);
}

void loop()
{
  // main FOC algorithm function
  
  if (millis () - loop_time > 1000){
    loop_rate = loop_count;
    loop_count = 0;
    loop_time = millis();
    Serial.println(loop_rate);
  }
  // Motion control function
  motor.move();
  // sensor.update();
  current_pos = -abs(sensor.getAngle());
  
  // Serial.println(starting_pos);
  // Serial.println(current_pos);
  // Serial.println(output);
  // Serial.println(sensor.getAngle() - starting_pos);
  // Serial.println();
  // if (abs(sensor.getAngle() - starting_pos) < 0.2)
  // {
  //     myPID.SetTunings(Kp2, Ki2, Kd2);
  // }
  // else {
  //     myPID.SetTunings(Kp, Ki, Kd);
  // }
   myPID.Compute();

  if(abs(sensor.getAngle() - starting_pos) < 0.3){
    output = 0;
  }
  if (sensor.getAngle() - starting_pos > 0)
  {
    //Serial.println(-output);
    motor.target = -output;
  }
  else
  {
    //Serial.println(output);
    motor.target = output;
  }
  // motor.target = -(sensor.getAngle() - starting_pos) * 0.3;
  //  function intended to be used with serial plotter to monitor motor variables
  //  significantly slowing the execution down!!!!
  //  motor.monitor();
  motor.loopFOC();
  // user communication
  command.run();
  loop_count++;
}