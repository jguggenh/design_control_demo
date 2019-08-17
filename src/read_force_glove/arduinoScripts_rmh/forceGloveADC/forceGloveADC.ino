#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Int16.h> 
#include <std_msgs/Int16MultiArray.h>  

#include <Wire.h>

// ros stuff
ros::NodeHandle forceGloveADCLeft;
std_msgs::Int16MultiArray forceGloveSignal;
ros::Publisher forceGlovePubL("forceGlovePubLeft", &forceGloveSignal);

// sampe rate ros
//std_msgs::Int16 stSignal;
//ros::Publisher stPub("stPub", &stSignal);

// pins for fsr
int fsrPinThumbL = 6;     // the FSR and 10K pulldown are connected to a0
int fsrPinIndexL = 7;     // the FSR and 10K pulldown are connected to a0
int fsrPinMiddleL = 4;     // the FSR and 10K pulldown are connected to a0
int fsrPinRingL = 5;
int fsrPinPinkyL = 2;

int prevTime = 0;
int currTime = 0;

void setup(void) 
{
  // initiate ros 
  forceGloveADCLeft.initNode();
  
  // define message
  forceGloveSignal.layout.dim[0].size = 2;
  forceGloveSignal.layout.dim[0].stride = 1*2;
  forceGloveSignal.data = (int *)malloc(sizeof(int)*2);
  forceGloveSignal.data_length = 5;
  
  //advertise signal
  forceGloveADCLeft.advertise(forceGlovePubL);

  //advertise sample rate pub
//  forceGloveA/C.advertise(stPub);
}
 
void loop(void) 
{
  // read force sensing resistor signal
  forceGloveSignal.data[0] = analogRead(fsrPinThumbL); 
  forceGloveSignal.data[1] = analogRead(fsrPinIndexL);  
  forceGloveSignal.data[2] = analogRead(fsrPinMiddleL); 
  forceGloveSignal.data[3] = analogRead(fsrPinRingL);  
  forceGloveSignal.data[4] = analogRead(fsrPinPinkyL); 

  

  // publish that value
  forceGlovePubL.publish(&forceGloveSignal);

  // ros stuff
  forceGloveADCLeft.spinOnce();

  //check sample rate
//  currTime = millis();
//  stSignal.data = currTime-prevTime;
//  prevTime = currTime;
//  stPub.publish(&stSignal);
} 
