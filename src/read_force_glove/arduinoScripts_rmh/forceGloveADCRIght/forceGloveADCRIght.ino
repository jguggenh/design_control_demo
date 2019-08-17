#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Int16.h> 
#include <std_msgs/Int16MultiArray.h>  

#include <Wire.h>

// ros stuff
ros::NodeHandle forceGloveADCRight;
std_msgs::Int16MultiArray forceGloveSignal;
ros::Publisher forceGlovePubR("forceGlovePubRight", &forceGloveSignal);

// sampe rate ros
//std_msgs::Int16 stSignal;
//ros::Publisher stPub("stPub", &stSignal);

// pins for fsr
int fsrPinThumbR = 6;     // the FSR and 10K pulldown are connected to a0
int fsrPinIndexR = 7;     // the FSR and 10K pulldown are connected to a0
int fsrPinMiddleR = 4;     // the FSR and 10K pulldown are connected to a0
int fsrPinRingR = 5;
int fsrPinPinkyR = 2;

int prevTime = 0;
int currTime = 0;

void setup(void) 
{
  // initiate ros 
  forceGloveADCRight.initNode();
  
  // define message
  forceGloveSignal.layout.dim[0].size = 2;
  forceGloveSignal.layout.dim[0].stride = 1*2;
  forceGloveSignal.data = (int *)malloc(sizeof(int)*2);
  forceGloveSignal.data_length = 5;
  
  //advertise signal
  forceGloveADCRight.advertise(forceGlovePubR);

  //advertise sample rate pub
//  forceGloveA/C.advertise(stPub);
}
 
void loop(void) 
{
  // read force sensing resistor signal
  forceGloveSignal.data[0] = analogRead(fsrPinThumbR); 
  forceGloveSignal.data[1] = analogRead(fsrPinIndexR);  
  forceGloveSignal.data[2] = analogRead(fsrPinMiddleR); 
  forceGloveSignal.data[3] = analogRead(fsrPinRingR);  
  forceGloveSignal.data[4] = analogRead(fsrPinPinkyR); 


  // publish that ow memoryvalue
  forceGlovePubR.publish(&forceGloveSignal);

  // ros stuff
  forceGloveADCRight.spinOnce();

  //check sample rate
//  currTime = millis();
//  stSignal.data = currTime-prevTime;
//  prevTime = currTime;
//  stPub.publish(&stSignal);
} 
