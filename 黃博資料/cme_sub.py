#!/usr/bin/env python3
import os
import cv2
import rospy
import numpy as np
# import IPM
import ME_decoder as ME
import IVA_decoder as IVA
import sys
import time
import datetime
import math

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from can_msgs.msg import Frame
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()

count = 0



def draw_obj(ipm,ME_output_obj,IVA_output_obj,w):
    
    me_TXT = []
    iva_TXT = []
    
    for i in range(len(ME_output_obj)):
        obj_x = int((ME_output_obj[i][1][0]*10) + (w/2))
        obj_y = int(ME_output_obj[i][1][1]*10)
        obj_w = int(ME_output_obj[i][0][1]*10)
        obj_h = int(ME_output_obj[i][0][2]*10)
        me_TXT.append("[" + str(ME_output_obj[i][1][0])+","+str(ME_output_obj[i][1][1])+']\n')
        cv2.rectangle(ipm, (int(obj_x-(7)), obj_y), (int(obj_x+(7)), int(obj_y+14)), (0, 150, 0), -1)
    
    # print("[IVA] get obj num = ", len(IVA_output_obj))
    for i in range(len(IVA_output_obj)):
        obj_x = int((IVA_output_obj[i][2]*10) + (w/2))
        obj_y = int(IVA_output_obj[i][3]*10)
        pts =  np.array([[obj_x-7, obj_y], [obj_x+7, obj_y], [obj_x, obj_y+14]], np.int32)
        iva_TXT.append("["+str(IVA_output_obj[i][2])+','+str(IVA_output_obj[i][3])+']\n')
        cv2.polylines(ipm, [pts], True, (0, 0, 255), 2)
        
    return ipm,iva_TXT,me_TXT

def new_IPMimg(h,w):
    ipm = np.zeros((h,w,3), dtype="uint8")+255
    
    for i in range(w//10):
        if (i == w/20):
            cv2.line(ipm, (int(i*10), 0), (int(i*10), h), (23, 0, 116), 2)
            
        elif (i%5 == 0):
            cv2.line(ipm, (int(i*10), 0), (int(i*10), h), (0, 0, 0), 2)
            
        else:
            cv2.line(ipm, (int(i*10), 0), (int(i*10), h), (90, 90, 90), 1)
            
    for i in range(h//10):
        if (i%10 == 0):
            cv2.line(ipm, (0,int(i*10)), (w, int(i*10)), (0, 0, 0), 2)
            
        elif (i%5 == 0):
            cv2.line(ipm, (0,int(i*10)), (w, int(i*10)), (23, 0, 116), 1)
            
        else:
            cv2.line(ipm, (0,int(i*10)), (w, int(i*10)), (90, 90, 90), 1)
            
    
    return ipm

def callback_cam(data):
    rospy.loginfo('CAM' + rospy.get_caller_id())
    
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.imshow("SUB_FRAME", frame)
    cv2.waitKey(1)
    
def callback_lidar(data):
    rospy.loginfo('Lidar' + rospy.get_caller_id())
    	
   



def Get_Me_CAN_data(CANdata):  
    
    img = np.zeros((150, 850, 3), np.uint8)
    img.fill(220)
    
    ######### ME_ID ##########
    
    ME_L_lane_id       = 1905  #0x771
    ME_L_info_id       = 1904  #0x770
    ME_R_lane_id       = 1907  #0x773
    ME_R_info_id       = 1906  #0x772
    
    
    ME_obj_num_id      = 1345  #0x541
    ME_obj_infoA_id    = 1280  #0x500
    ME_obj_infoB_id    = 1281  #0x501
    ME_obj_infoC_id    = 1282  #0x502
    
    ME_fail_safes      = 1680  #0x690
    ME_car_signals     = 1888  #0x760
    ME_Gyro_data_id    = 1795  #0x703  #陀螺儀計算yaw/roll/pitch角度
    ME_display_warnings= 1792  #0x700
    
    draw_obj = []
    obj_tmp  = []
    output_obj = []
    obj_index = 0
    obj_num = 0
    ######### ME_ID ##########
    
    if CANdata.data:
        if (obj_index > 11):
            obj_index = 11
        if (CANdata.id == ME_L_lane_id):
            text = 'receive ME canbus : {} ' .format(ME_L_lane_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_L_info_id):
            text = 'receive ME canbus : {} ' .format(ME_L_info_id)
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_R_lane_id):
            text = 'receive ME canbus : {} ' .format(ME_R_lane_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_R_info_id):
            text = 'receive ME canbus : {} ' .format(ME_R_info_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_obj_num_id):
            obj_num = CANdata.data[1] 
            obj_num = int(obj_num%16)
            print('ME_obj_number:{}' .format(obj_num))
            text = 'receive ME canbus : {} ,objNUM : {}' .format(ME_obj_num_id,obj_num) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == (ME_obj_infoA_id+(obj_index*3))):

            obj_tmp.append(ME.Q4_Obj_infoA_decoder(list(CANdata.data)))
            text = 'receive ME canbus : {} ' .format(ME_obj_infoA_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == (ME_obj_infoB_id+(obj_index*3))):

            obj_tmp.append(ME.Q4_Obj_infoB_decoder(list(CANdata.data)))
            text = 'receive ME canbus : {} ' .format(ME_obj_infoB_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == (ME_obj_infoC_id+(obj_index*3))):

            obj_tmp.append(ME.Q4_Obj_infoC_decoder(list(CANdata.data)))
            text = 'receive ME canbus : {} ' .format(ME_obj_infoC_id) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
            if (len(obj_tmp)==3):
                draw_obj.append(obj_tmp)
                obj_index = obj_index + 1
                
            obj_tmp = []
            
        elif (CANdata.id == ME_fail_safes):
            text = 'receive ME canbus : {} ' .format(ME_fail_safes)
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_car_signals):
            text = 'receive ME canbus : {} ' .format(ME_car_signals) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_Gyro_data_id):
            ME_angle_yaw,ME_angle_roll,ME_angle_pitch = ME.Q4_Gyro_rate_data(CANdata.data)
            text = 'receive ME canbus : {} ,Pitch_angle : {}' .format(ME_Gyro_data_id,ME_angle_pitch) 
            cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
            
        elif (CANdata.id == ME_display_warnings):
            ME_warning_data=ME.Q4_warning(CANdata.data)

            for i  in range(int(len(ME_warning_data))):
                if ME_warning_data[i]:
                    
                    if i==0:
                        warning_type='FCW'                    
                        
                    elif i==1:
                        warning_type='PCW'                    
                        
                    elif i==2:
                        warning_type='L_LDW'                  
                        
                    elif i==3:
                        warning_type='R_LDW'
                                         
                    elif i==4:
                        warning_type='ped_in_dangerzone'
                    
                    elif i==5:
                        warning_type='HW_warning_level'
                        
                    elif i==6:
                        warning_type='sound_type'

                    text = 'receive ME canbus : {}_{}' .format(CANdata.id,warning_type) 
                    cv2.putText(img, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, (50, 205, 50), 1, cv2.LINE_AA)
                    
        cv2.imshow("SUB_CAN_FRAME", img)
        cv2.waitKey(1)
        
                    
        if (obj_index == obj_num):
            print('ME_obj:{}'.format(obj_num))
            output_obj = draw_obj 
            draw_obj = []
            obj_index = 0
            
    return output_obj
    
def Get_IVA_CAN_data(CANdata):
    
    img = np.zeros((150, 850, 3), np.uint8)
    img.fill(220)
    
    ######### IVA_ID ##########
    IVA_warning_ID = 0x4E1  #1249
    IVA_L_1st_ID = 0x4E2   #1250
    IVA_R_1st_ID = 0x4E3   #1251
    IVA_L_2nd_ID = 0x4E4   #1252
    IVA_R_2nd_ID = 0x4E5   #1253
    IVA_lane_dis_ID = 0x4E6  #1254
    IVA_obj_info_ID = 0x4E7  #1255
    IVA_obj_num_ID = 0x4E8   #1256
    IVA_L_1st_info_ID = 0x4E9  #1257
    IVA_R_1st_info_ID = 0x4EA  #1258
    IVA_L_2nd_info_ID = 0x4EB  #1259
    IVA_R_2nd_info_ID = 0x4EC  #1260
    IVA_L_roadside_ID = 0x4ED  #1261
    IVA_R_roadside_ID = 0x4EE  #1262
    
    ######### IVA_ID ##########
    L_curve = [0,0,0,0,0,0] 
    R_curve = [0,0,0,0,0,0] 
    
    Lane_info = [0,0,0,0,0,0,0,0]
    
    obj_tmp  = []
    # draw_obj = []
    output_obj = []
    obj_index = 0
    obj_num = 0
    

    if (CANdata.id == IVA_warning_ID):
        print('CAN.id={}'.format(CANdata.id ))
        IVA_warning_data=IVA.IVA_warning_type(CANdata.data)

        for i  in range(int(len(IVA_warning_data))):
            if IVA_warning_data[i]:
                
                if i==0:
                    warning_type='VB'                    
                    
                elif i==1:
                    warning_type='PD'                    
                    
                elif i==2:
                    warning_type='L_LDW'                  
                    
                elif i==3:
                    warning_type='R_LDW'
                                     
                elif i==4:
                    warning_type='FCW'
                
                elif i==5:
                    warning_type='TTC_time'
                
                text = 'receive IVA canbus : {}_{}' .format(CANdata.id,warning_type) 
                cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_L_1st_ID):

        text = 'receive IVA canbus : {} ' .format(IVA_L_1st_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_R_1st_ID):

        text = 'receive IVA canbus : {} ' .format(IVA_R_1st_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_L_2nd_ID):

        text = 'receive IVA canbus : {} ' .format(IVA_L_2nd_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_R_2nd_ID):

        text = 'receive IVA canbus : {} ' .format(IVA_R_2nd_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_lane_dis_ID):

        text = 'receive IVA canbus : {} ' .format(IVA_lane_dis_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_obj_info_ID):

        obj_tmp.append(IVA.IVA_obj_decode(list(CANdata.data)))
        obj_index = obj_index+1
        text = 'receive IVA canbus : {} ' .format(IVA_obj_info_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_obj_num_ID):
        obj_num=CANdata.data[0]
        print('IVA_obj_number:{}' .format(obj_num))
        text = 'receive IVA canbus : {} ,objNUM : {}' .format(IVA_obj_num_ID,obj_num) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_L_1st_info_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_L_1st_info_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_R_1st_info_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_R_1st_info_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_L_2nd_info_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_L_2nd_info_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_R_2nd_info_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_R_2nd_info_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
    
    elif (CANdata.id == IVA_L_roadside_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_L_roadside_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    elif (CANdata.id == IVA_R_roadside_ID):
        text = 'receive IVA canbus : {} ' .format(IVA_R_roadside_ID) 
        cv2.putText(img, text, (10, 80), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 1, cv2.LINE_AA)
        
    cv2.imshow("SUB_CAN_FRAME", img)
    cv2.waitKey(1)
    
    
    #if (obj_index == obj_num):

    output_obj = obj_tmp
     #   obj_tmp = []
     #   obj_index = 0                  
                
        
    return output_obj

def callback_canbus(data):
    #rospy.loginfo('CANBUS' + rospy.get_caller_id()) 
    

    
    h = 800
    w = 200
    osd_w = 1280
    osd_h = 800
    
    ipm_ori = new_IPMimg(h,w)
    


        
    ME_output_obj = Get_Me_CAN_data(data)   
    IVA_output_obj = Get_IVA_CAN_data(data)


          
        #ipm = ipm_ori.copy() 
           

    ipm,iva_TXT,me_TXT = draw_obj(ipm_ori,ME_output_obj,IVA_output_obj,w)
        
    ipm = cv2.flip(ipm,0)
        
        
    cv2.imshow("IPM", ipm)
        
    cv2.waitKey(1) 

def callback_convert(data):
    rospy.loginfo('CONVERT' + rospy.get_caller_id()) 
    #print(data) 
    
def listener():
    rospy.init_node('cme_listen_node', anonymous=True)
    
    # sub_cam = rospy.Subscriber('cme_cam', Image, callback_cam)
    # sub_lidar = rospy.Subscriber('/rslidar_points', PointCloud2, callback_lidar)
    sub_canbus = rospy.Subscriber('/can0/received_msg', Frame, callback_canbus)
    # sub_canbus = rospy.Subscriber('/can2/received_msg', Frame, callback_canbus)
    # sub_canbus = rospy.Subscriber('cme_IVAcanbus', Frame, callback_canbus)
    # sub_canbus = rospy.Subscriber('cme_MEcanbus', Frame, callback_canbus)
    # sub_convert = rospy.Subscriber('cme_convert', MarkerArray, callback_convert)
    
    rospy.spin()
    
if __name__ == '__main__':
    listener()

