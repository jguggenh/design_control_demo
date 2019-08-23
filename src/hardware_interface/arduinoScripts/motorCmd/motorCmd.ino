#include "motorClass.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include "DualMC33926MotorShield.h"

// Robotzone motors
float z_actuator_gear_ratio = 71;
float z_actuator_enc_cnts_per_rev = 48.0;
float roll_actuator_gear_ratio = 71;
float roll_actuator_enc_cnts_per_rev = 48.0;

// create motor objects
// motorClass::motorClass(int pwmPin,int dirPin, int encPin,float gearRatio, float encCntsRev);
motorClass z_actuator =  motorClass(5,3,8,z_actuator_gear_ratio,z_actuator_enc_cnts_per_rev); 
motorClass roll_actuator =  motorClass(6,4,2,roll_actuator_gear_ratio,roll_actuator_enc_cnts_per_rev);

// gripper motor stuff
DualMC33926MotorShield md;
int gripperSpeed = 100;

// ros stuff
ros::NodeHandle arduino2Motor;

// publisher for testing
std_msgs::Float32 testing;
ros::Publisher testingPub("testing", &testing);

// home and open door position for actuators
float z_actuator_home_pos = 0; //TODO
float z_actuator_open_door_pos = -0.189; //TODO
float roll_actuator_home_pos = 0; //TODO
float roll_actuator_open_door_pos = 0.50; //TODO

void z_actuator_callback(const std_msgs::Float32& command)
{
	// go home
	if (command.data == 0)
	{
		z_actuator.setMotorPos(z_actuator_home_pos);
	}
	
	// go to open door position
	if (command.data == 1)
	{
		z_actuator.setMotorPos(z_actuator_open_door_pos);
	}
}

void roll_actuator_callback(const std_msgs::Float32& command)
{
	// go home
	if (command.data == 0)
	{
		roll_actuator.setMotorPos(roll_actuator_home_pos);
	}
	
	// go to open door position
	if (command.data == 1)
	{
		roll_actuator.setMotorPos(roll_actuator_open_door_pos);
	}
}

void gripper_callback(const std_msgs::Float32& command)
{
  // go home
  if (command.data == 0)
  {
    md.setM1Speed(100);
    testing.data = 0;
    testingPub.publish(&testing);
  }
  else if (command.data == 1) // go to open door position
  {
    md.setM1Speed(-100);
    testing.data = 1;
    testingPub.publish(&testing);
  }
  else // stop moving!
  {
    md.setM1Speed(0);
    testing.data = 2;
    testingPub.publish(&testing);
  }
}


// robotzone subscribers
ros::Subscriber<std_msgs::Float32> z_actuator_command_sub("z_actuator_command", &z_actuator_callback);
ros::Subscriber<std_msgs::Float32> roll_actuator_command_sub("roll_actuator_command", &roll_actuator_callback);
ros::Subscriber<std_msgs::Float32> gripper_command_sub("gripper_command", &gripper_callback);


void setup () 
{ 
  // initialize ros
  arduino2Motor.initNode();
  
  // subscriber
  arduino2Motor.subscribe(z_actuator_command_sub);
  arduino2Motor.subscribe(roll_actuator_command_sub);
  arduino2Motor.subscribe(gripper_command_sub);

  // advertise testing publisher
  arduino2Motor.advertise(testingPub);

//  Serial.begin(57600);  
//  forceMotor.setMotorForce(1);

  // gripper
  md.init();

  delay(1000);
}

int printing = 0;
void loop ()
{
  if (printing > 1000)
  {  
	  // send testing
	  testing.data = z_actuator.MotorPos;
	  testingPub.publish(&testing);

	  // Serial.println(forceMotor.MotorForce);

	  // reset
	  printing = 0; 
  }
  
  // control robotzone motors
  z_actuator.pos_closedLoopController();
  roll_actuator.pos_closedLoopController();

  printing = printing + 1;

  arduino2Motor.spinOnce();
}
