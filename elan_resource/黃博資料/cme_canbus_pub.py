#!/usr/bin/env python3
# import IPM
import ME_decoder as ME
import IVA_decoder as IVA
import sys
import os
import cv2
import rospy
import time
import datetime
#from canlib import canlib
#from canlib import Frame as canFrame
#import canlib
from can_msgs.msg import Frame
from std_msgs.msg import String
import canalystii


if __name__ == '__main__':

    rospy.init_node('cme_IVAnode', anonymous=True)
    canbus_pub = rospy.Publisher('cme_IVAcanbus', Frame, queue_size=0)
    
    rate = rospy.Rate(100)  #數字改掉,10ms收一次會不會重複,找到不會重複的時間
    
    #channels = canlib.getNumberOfChannels()
    #for dev in canlibl.connected_devices():
    #    print(devprobe_info())
    
    #cl = canlib.openChannel(channel=0, flags=canlib.Open.EXCLUSIVE, bitrate=canlib.Bitrate.BITRATE_500K)
    cl =canalystii.CanalystDevice(bitrate=500000)
    

    #cl.setBusOutputControl(canlib.Driver.NORMAL)
    #cl.busOn()
    
    #print("CANBUS Device number: %d" % channels)

    
    print("[CME] IVA CANBUS publisher init ok")
    while not rospy.is_shutdown():
        #msg = ch_a.read(timeout=500)
        #try:

            #recvFrame = cl.read(timeout=100)
            #ret, frame = video_cap.read()
            
            #if recvFrame.id == 1256 or recvFrame.id == 1255:
            #    print(recvFrame)
                
#        frame = canFrame(id_=123, data=b'HELLO!', dlc=6)
#        cl.write(frame)
#        cl.writeSync(timeout=500)
       
#        msg = "CANBUS : %s" % rospy.get_time()
                   
            '''
            for msg in cl.receive(0) :
                can_msgME = Frame()#(id=msg.can_id,is_extended=msg.extended,dlc=msg.data_len,data=msg.data)
                can_msgME.id=msg.can_id
                can_msgME.is_extended=msg.extended
                can_msgME.dlc=msg.data_len
                can_msgME.data=(list(msg.data))
              
                canbus_pub.publish(can_msgME)
            '''
            
            for msg in cl.receive(1) :
                can_msgIVA = Frame()#(id=msg.can_id,is_extended=msg.extended,dlc=msg.data_len,data=msg.data)
                can_msgIVA.id=msg.can_id
                can_msgIVA.is_extended=msg.extended
                can_msgIVA.dlc=msg.data_len
                can_msgIVA.data=(list(msg.data)) 
                canbus_pub.publish(can_msgIVA)
            
            #for i in range(0, recvFrame.dlc):
            #	can_msg.data[i] = i #recvFrame.data.pop()
            #can_msg.data = frame.data
            
                

            #if recvFrame.id == 1256 or recvFrame.id == 1255:
            #    print(can_msg)
            #rate.sleep()
            
        #except( canlib.canNoMsg ) as ex:
        #    None
        #except ( canlib.canError ) as ex:
        #    print(ex)

    #cl.busOff()

    # Close the channel.
    #cl.close()