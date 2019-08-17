#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32
import time
#from statistics import mean
import numpy as np

#roslaunch dual_force_glove_launch.launch
#subscribe to /CommandCode

class ForceGloveInterpreter():
    def __init__(self):
        rospy.init_node('ForceGloveListener', anonymous=True)

        rospy.Subscriber("/forceGlovePubRight", Int16MultiArray, self.callbackRight)
        rospy.Subscriber("/forceGlovePubLeft", Int16MultiArray, self.callbackLeft)

        self.pub = rospy.Publisher('CommandCode', Int32, queue_size=10)

        self.allzeros=[0,0,0,0,0,0,0,0,0,0]
        self.vals1=[0,0,0,0,0]
        self.vals2=[0,0,0,0,0]
        self.vals3=[0,0,0,0,0,0,0,0,0,0]
        self.offsets=[0,0,0,0,0,0,0,0,0,0]
        self.binary=[0,0,0,0,0,0,0,0,0,0]
        self.counter=0
        #edit or add commands here, make sure you update the code farther down if you add codes
        self.command1=[1,0,1,1,1,1,1,1,1,1]
        self.command2=[1,1,1,1,1,1,0,1,1,1]
        self.command3=[1,1,0,1,1,1,1,1,1,1]
        self.command4=[1,1,1,1,1,1,1,0,1,1]
        self.command5=[1,1,1,0,0,1,1,1,1,1]
        self.command6=[1,1,1,1,1,1,1,1,0,0]
        self.commandtest=[0,0,0,0,0,0,0,0,0,1]
        self.commandtopub=0
        self.rate=rospy.Rate(50)
        self.counter_target=20

        self.prevdata=[0,0,0,0,0,0,0,0,0,0]
    def mean(self,lst):
        return sum(lst) / len(lst)

    def callbackLeft(self,data):
        #print( data.data)
        self.vals1=data.data

    def callbackRight(self,data):
        #print( data.data)
        self.vals2=data.data

    def calibrate(self):
        self.calibratetime=3
        timeout=time.time()+self.calibratetime
        L1=[]
        L2=[]
        L3=[]
        L4=[]
        L5=[]
        R1=[]
        R2=[]
        R3=[]
        R4=[]
        R5=[]
        print('Calibrating'        )
        while time.time() < timeout:
                    L1.append(self.vals1[0])
                    L2.append(self.vals1[1])
                    L3.append(self.vals1[2])
                    L4.append(self.vals1[3])
                    L5.append(self.vals1[4])
                    R1.append(self.vals2[0])
                    R2.append(self.vals2[1])
                    R3.append(self.vals2[2])
                    R4.append(self.vals2[3])
                    R5.append(self.vals2[4])
        self.offsets=np.array([self.mean(L1),self.mean(L2),self.mean(L3),self.mean(L4),self.mean(L5),self.mean(R1),self.mean(R2),self.mean(R3),self.mean(R4),self.mean(R5)])
        print('Done Calibrating')

    def getbinary(self):
        #print('vals2',self.vals2)
        self.vals3=np.array([self.vals1 + self.vals2])
        #print('vals3',self.vals3)
        #print('offsets',self.offsets)
        self.adjusted=np.subtract(self.vals3,self.offsets)
        #print('selfadjusted',self.adjusted)
        #print('len',len(self.adjusted[0]))
        for x in range(0,len(self.adjusted[0])):
            if self.adjusted[0][x] >= 100:
                self.binary[x]=1
            else:
                self.binary[x]=0

        #print('selfbinary',self.binary)
        #print(self.vals1,self.vals2)

    def getcode(self):
        self.getbinary()
        #self.commandtopub=0
        #self.command1=[1,0,1,1,1,1,1,1,1,1]
        #self.command2=[1,1,1,1,1,1,0,1,1,1]
        #self.command3=[1,1,0,1,1,1,1,1,1,1]
        #self.command4=[1,1,1,1,1,1,1,0,1,1]
        #self.command5=[1,1,1,0,0,1,1,1,1,1]
        #self.command6=[1,1,1,1,1,1,1,1,0,0]
        if self.prevdata==self.binary and self.binary != self.allzeros:
            self.counter=self.counter+1

        else:
            self.counter=0
            self.commandtopub=0
        self.prevdata=self.binary
        #IF YOU ADD COMMANDS ALSO ADD THEM HERE WITH A CODE
        if self.counter>=self.counter_target:
            if self.binary==self.command1:
                self.commandtopub=1
            if self.binary==self.command2:
                self.commandtopub=2
            if self.binary==self.command3:
                self.commandtopub=3
            if self.binary==self.command4:
                self.commandtopub=4
            if self.binary==self.command5:
                self.commandtopub=5
            if self.binary==self.command6:
                self.commandtopub=6
            if self.binary==self.commandtest:
                self.commandtopub=7

        print(self.binary,self.counter,self.commandtopub)



    def mymain(self):
        while not rospy.is_shutdown():
            self.getcode()
            self.pub.publish(self.commandtopub)
            self.rate.sleep()

if __name__ == '__main__':
    b=ForceGloveInterpreter()
    b.calibrate()
    b.mymain()
