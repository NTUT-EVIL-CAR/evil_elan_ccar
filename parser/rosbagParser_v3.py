import numpy as np
import os
import rosbag
import cv2
import shutil
import folium
import subprocess
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
import open3d as o3d

def prettyhex2(dlc,nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums[0:dlc])

def get_curve(_C_L,_C_R,L_max_dis,R_max_dis):
    Y_L =  np.linspace(0,L_max_dis,720) # distance
    Y_R =  np.linspace(0,R_max_dis,720) # distance
    X_R,X_L =0,0

    for i in range(4):
        X_L = X_L + (_C_L[i]*(Y_L**i))
        X_R = X_R + (_C_R[i]*(Y_R**i))
        
    return X_L,X_R,Y_L,Y_R

def get_curve1(_C_R,R_max_dis):
    Y_R =  np.linspace(0,R_max_dis,720) # distance
    X_R,X_L =0,0

    for i in range(4):
        X_R = X_R + (_C_R[i]*(Y_R**i))
        
    return X_R,Y_R

def point2IPM (xs, ys):
    point_X,point_Y = [],[]
    sin_theta = 0.041791174430386
    cos_theta = 0.999126367252776
    Camera_Height = 13.400000000000000 # 10     cm/per
    Camera_Height_sin_theta = 0.560001737367174
    Camera_Height_cos_theta = 13.388293321187199
    Kx = 1667.257003491107071
    Ky = 1721.346660030468001
    input_image_width = 1280
    input_image_height = 720
    # x_pix=10cm

    for x, y in zip(xs, ys):
        out_y = Camera_Height *(Ky * cos_theta -(y - (input_image_height/2)) *sin_theta) /(Ky * sin_theta +(y - (input_image_height >> 1)) *cos_theta)
        out_x = (x - (input_image_width/2)) *(out_y * cos_theta - Camera_Height_sin_theta) /Kx
        point_X.append(out_x)
        point_Y.append(out_y)
    
    return point_X,point_Y

def IPM2point (xs, ys):
    point_X,point_Y = [],[]
    sin_theta = 0.041791174430386
    cos_theta = 0.999126367252776
    Camera_Height = 13.400000000000000
    Camera_Height_sin_theta = 0.560001737367174
    Camera_Height_cos_theta = 13.388293321187199
    Kx = 1667.257003491107071
    Ky = 1721.346660030468001
    input_image_width = 1280
    input_image_height = 720
    
    for x, y in zip(xs, ys):
        out_y = (((y * Ky * sin_theta)-(Camera_Height * (Ky * cos_theta)))/((y*cos_theta)-(Camera_Height*sin_theta)))+(input_image_height >> 1)
        out_y = input_image_height-out_y
        out_x = ((Kx*x) / ((y*cos_theta)-Camera_Height_sin_theta))+(input_image_width >> 1)
        point_X.append(out_x)
        point_Y.append(out_y)

    return point_X,point_Y

def get_two_float(f_str, n):
    f_str = str(f_str)
    a, b, c = f_str.partition('.')
    c = (c+"0"*n)[:n]

    return ".".join([a, c])

def prettyhex(nums, sep=''):

    return sep.join(f'{a:02x}' for a in nums)

def IVA_obj_290B(num):
    ID = int(num[0],16) & 0x3F
    classes = ((int(num[0],16)>>6) + (int(num[1],16)<<2)) & 0x1F
    conf = ((int(num[1],16)>>3) + (int(num[2],16)<<5)) & 0x7F
    conf = conf * 0.78125
    Y = ((int(num[2],16)>>2) + (int(num[3],16)<<6)) & 0xFFF
    Y = Y * 0.03125
    X = ((int(num[3],16)>>6) + (int(num[4],16)<<2) + (int(num[5],16)<<10)) & 0xFFF
    X = X * 0.015625 -32
    vy = ((int(num[5],16)>>2) + (int(num[6],16)<<6)) & 0x7FF
    vy = vy * 0.019531 - 20
    vx = ((int(num[6],16)>>5) + int(num[7],16)) & 0x7FF

    return [ID, classes, conf, X, Y, vx ,vy]

def IVA_obj_290D(num):
    objnum = int(num[0],16) & 0xFF
    lms=[]

    for i in range(5):
        lm0 = int(num[i],16) & 0x03
        lm1 = (int(num[i],16)>>2) & 0x03
        lm2 = (int(num[i],16)>>4) & 0x03
        lm3 = (int(num[i],16)>>6) & 0x03
        lms.extend([lm0,lm1,lm2,lm3])

    return [objnum, lms]

def IVA_Lane_Distance_decode(can_data):
    dis_L_1st = int(can_data[0], 16)
    dis_R_1st = int(can_data[1], 16)
    type_L_1st = int(can_data[2], 16) & 0x0F
    type_R_1st = (int(can_data[2], 16) & 0xF0) >> 4
    color_L_1st = int(can_data[3], 16) & 0x0F
    color_R_1st = (int(can_data[3], 16) & 0xF0) >> 4
    
    dis_L_2nd = int(can_data[4], 16)
    dis_R_2nd = int(can_data[5], 16)
    type_L_2nd = int(can_data[6], 16) & 0x0F
    type_R_2nd = (int(can_data[6], 16) & 0xF0) >> 4
    color_L_2nd = int(can_data[7], 16) & 0x0F
    color_R_2nd = (int(can_data[7], 16) & 0xF0) >> 4
    
    return dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, color_L_1st, color_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd, color_L_2nd, color_R_2nd

def IVA_lanecurve_2902468(num):
    # type and color
    fit_score_far = int(num[0],16)
    fit_score_mid = int(num[1],16)
    fit_score_close = int(num[2],16)
    distance_score = int(num[3],16)
    detect_score = int(num[4],16)
    
    return  fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score

def IVA_lanecurve_2901379(num, flag):
    quality = int(num[0],16) & 0x07
    exist = int(num[0],16)>>3 & 0x01

    if flag == '7':
        c0 = ((int(num[0],16)>>4) + (int(num[1],16)<<4)) * 0.003906 - 18
    elif flag == '9':
        c0 = ((int(num[0],16)>>4) + (int(num[1],16)<<4)) * 0.003906 + 2
    else:
        c0 = ((int(num[0],16)>>4) + (int(num[1],16)<<4)) * 0.003906 - 8
    c1 = (int(num[2],16) + (int(num[3],16)<<8)) * 0.00001089 - 0.357
    c2 = (int(num[4],16) + (int(num[5],16)<<8)) * 0.00000061037 - 0.02
    c3 = (int(num[6],16) + (int(num[7],16)<<8)) * 0.00000000366217 - 0.00012
    
    # return quality, exist, [c0, c1, c2, c3]
    return quality, exist, c0, c1, c2, c3

def IVA_obj_num_decode(num):
    number = num[0]
    
    return [number]

def rot(a,theta):

    return [a[1]*np.sin(theta)+a[0]*np.cos(theta),  a[1]*np.cos(theta)-a[0]*np.sin(theta)]

def Q4_541(num):
    objnum = int(num[1],16) & 0x0F
    CIPV = ((int(num[1],16)>>4) + (int(num[2],16)<<4)) & 0x7F
    CIPVlost = (int(num[2],16)>>3) & 0x03
    acc = (int(num[2],16)>>5) & 0x03

    return objnum, CIPV, CIPVlost, acc

def Q4_500(num):
    ID = int(num[0],16) & 0x7F
    cls = ((int(num[0],16)>>7) + (int(num[1],16)<<1)) & 0x07
    w = (((int(num[1],16)>>2) + (int(num[2],16)<<6)) & 0x7F) * 0.05
    l = (((int(num[2],16)>>1) + (int(num[3],16)<<7)) & 0x1FF) * 0.05
    relative_long_v = (((int(num[3],16)>>2) + (int(num[4],16)<<6)) & 0x1FFF) * 0.05 - 120
    obj_lane_assignment = ((int(num[4],16)>>7) + (int(num[5],16)<<1)) & 0x07
    relative_lat_v = (((int(num[5],16)>>2) + (int(num[6],16)<<6)) & 0x7FF) * 0.05 - 20
    
    return [ID,cls,w,l,relative_long_v, obj_lane_assignment, relative_lat_v]

def Q4_501(num):
    abs_long_acc = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x1FF) * 0.05 - 12.8
    long_dis = (((int(num[1],16)>>1) + (int(num[2],16)<<7)) & 0x1FFF) * 0.05
    lat_dis = (((int(num[2],16)>>6) + (int(num[3],16)<<2) + (int(num[4],16)<<10)) & 0xFFF) * 0.05 - 102.4
    # long_dis_M = (((int(num[4],16)>>2) + (int(num[5],16)<<6)) & 0x1FFF) * 0.05    #bag error
    
    return [abs_long_acc, long_dis, lat_dis]

def Q4_502(num):
    abs_speed = ((int(num[0],16) + (int(num[1],16)<<8)) & 0xFFF) * 0.05 -100
    ms_status = (int(num[1],16)>>4) & 0x07
    ms_cate   = ((int(num[1],16)>>7) + (int(num[2],16)<<1)) & 0x0F
    brake     = (int(num[2],16)>>3) & 0x01
    turnR     = (int(num[2],16)>>4) & 0x01
    turnL     = (int(num[2],16)>>5) & 0x01
    light_valid  = (int(num[2],16)>>6) & 0x01
    obj_angle_mid = (((int(num[2],16)>>7) + (int(num[3],16)<<1) + (int(num[4],16)<<9)) & 0x3FFF) * 0.0002 - 1.571
    
    if len(num)<7:
        obj_angle_rate = 0
    else:
        obj_angle_rate = (((int(num[4],16)>>5) + (int(num[5],16)<<3) + (int(num[6],16)<<11)) & 0xFFF) * 0.002 - 2.234
    
    return [abs_speed, ms_status, ms_cate, brake, turnR, turnL, light_valid, obj_angle_mid, obj_angle_rate]

def Q4_76E(num):
    
    return int(num[0],16) & 0x01

def Q4_770_772(num):
    LH_conf = int(num[0],16) & 0x7F
    LH_landmark_type = ((int(num[0],16)>>7) + (int(num[1],16)<<1)) & 0x0F
    LH_side = (int(num[1],16) >>3) & 0x03
    LH_VR_start = (((int(num[1],16)>>5) + (int(num[2],16)<<3) + (int(num[3],16)<<11)) & 0x7FFF) * 0.01
    LH_VR_end = (((int(num[3],16)>>4) + (int(num[4],16)<<4) + (int(num[5],16)<<12)) & 0x7FFF) * 0.01
    
    return [LH_conf, LH_landmark_type, LH_side, LH_VR_start, LH_VR_end]

def Q4_771_773(num):
    c0 = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x7FF) * 0.01 - 10
    c1 = (((int(num[1],16)>>3) + (int(num[2],16)<<5)) & 0x3FF) * 0.000977 - 0.357
    c2 = (((int(num[2],16)>>5) + (int(num[3],16)<<3) + (int(num[4],16)<<11)) & 0xFFFF) * 0.000000977 - 0.032
    c3 = (((int(num[4],16)>>5) + (int(num[5],16)<<3) + (int(num[6],16)<<11)) & 0xFFFF) * 0.00000000373 - 0.000122
    
    return [c0, c1, c2, c3] 

def Q4_782(num):
    lane_adj_count = (int(num[0],16)>>4) & 0x0F
    
    return lane_adj_count

def Q4_78468A(num):
    typ = int(num[0],16) & 0x0F
    conf = (((int(num[0],16)>>4) + (int(num[1],16)<<4)) & 0x7F) * 0.01
    VR_start = (((int(num[1],16)>>3) + (int(num[2],16)<<5) + (int(num[3],16)<<13)) & 0x7FFF) * 0.01
    VR_end = (((int(num[3],16)>>2) + (int(num[4],16)<<6) + (int(num[5],16)<<14)) & 0x7FFF) * 0.01
    line_role = (int(num[5],16)>>1) & 0x0F
    
    return [typ, conf, VR_start, VR_end, line_role] 

def Q4_78579B(num):
    c0 = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x7FF) * 0.01 - 10
    c1 = (((int(num[1],16)>>3) + (int(num[2],16)<<5)) & 0x3FF) * 0.000977 - 0.357
    c2 = (((int(num[2],16)>>5) + (int(num[3],16)<<3) + (int(num[4],16)<<11)) & 0xFFFF) * 0.000000977 - 0.032
    c3 = (((int(num[4],16)>>5) + (int(num[5],16)<<3) + (int(num[6],16)<<11)) & 0xFFFF) * 0.00000000373 - 0.000122
    
    return [c0, c1, c2, c3] 

def Q4_690(num):
    ver = int(num[0],16) & 0xFF
    frameid = int(num[1],16) & 0xFF
    freesight = int(num[2],16) & 0x01
    rain = (int(num[2],16)>>1) & 0x07
    fog  = (int(num[2],16)>>4) & 0x07
    splash = ((int(num[2],16)>>7) + (int(num[3],16)<<1)) & 0x07
    sunray = (int(num[3],16)>>2) & 0x07
    lowsun = (int(num[3],16)>>5) & 0x07
    blurimg = int(num[4],16) & 0x07
    partial_blockage = (int(num[4],16)>>3) & 0x07
    full_blockage = ((int(num[4],16)>>6) + (int(num[5],16)<<2)) & 0x07
    frozen_blockage = (int(num[5],16)>>1) & 0x07
    outof_calib = (int(num[5],16)>>4) & 0x07
    outof_focus = ((int(num[5],16)>>6) + (int(num[6],16)<<1)) & 0x07
    TSR_outof_focus = ((int(num[6],16)>>2) + (int(num[7],16)<<6)) & 0xFF
    
    return [ver, frameid, freesight, rain, fog, splash, sunray, lowsun, blurimg, partial_blockage, full_blockage, frozen_blockage, outof_calib, outof_focus, TSR_outof_focus]

def Q4_486(num):
    img_timestamp = (int(num[1],16) + (int(num[2],16)<<8) + (int(num[3],16)<<16) + (int(num[4],16)<<24)) & 0xFFFFFFFF
    hzd_objnum = int(num[5],16) & 0x7F
    valid_frame = (int(num[5],16)<<6) & 0x01
    
    return [img_timestamp, hzd_objnum, valid_frame]

def Q4_487(num): 
    UTC_timestamp = int(num[0],16) + (int(num[1],16)<<8) + (int(num[1],16)<<16) + (int(num[3],16)<<24) + (int(num[4],16)<<32) + (int(num[5],16)<<40) + (int(num[6],16)<<48)
    
    return UTC_timestamp

def Q4_488ACE(num):
    hzd_id   = int(num[0],16) & 0xFF
    hzd_prob = ((int(num[1],16) + (int(num[2],16)<<8)) & 0x3FF)
    hzd_latpos  = ((((int(num[2],16)>>2) + (int(num[3],16)<<6) + (int(num[4],16)<<14)) & 0x7FFF)) 
    hzd_longpos = ((((int(num[4],16)>>1) + (int(num[5],16)<<7)) & 0x7FFF))
    hzd_vertpos = ((int(num[6],16) + (int(num[7],16)<<8)) & 0xFFF) 
    
    return [hzd_id, hzd_prob, hzd_latpos, hzd_longpos, hzd_vertpos]

def Q4_489BDF(num):
    hzd_h = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x1FF)
    hzd_w = (((int(num[1],16)>>1) + (int(num[2],16)<<7)) & 0x1FF) 
    hzd_latstd = (((int(num[2],16)>>6) + (int(num[3],16)<<2)) & 0xFF) 
    hzd_longstd = (((int(num[3],16)>>6) + (int(num[4],16)<<2)) & 0x1FF) 
    
    return [hzd_h, hzd_w, hzd_latstd, hzd_longstd]

def Q4_703(num):
    avai = int(num[0],16)>>7
    yaw = ((int(num[2],16) + (int(num[1],16)<<8)) & 0xFFFF)
    yaw = convert2signint(yaw, 16) * (-0.00875)
    raw = ((int(num[4],16) + (int(num[3],16)<<8)) & 0xFFFF)
    raw = convert2signint(raw, 16)* (-0.00875)
    pitch = ((int(num[6],16) + (int(num[5],16)<<8)) & 0xFFFF)
    pitch = convert2signint(pitch, 16)* (-0.00875)
    
    return [avai,pitch,raw,yaw]

def convert2signint(value,bits):
    if value & (1<< (bits-1)):
        value -= 1<< bits
    
    return value

def Q4_760(num):
    brake      = int(num[0],16) & 0x01
    Lblinker   = (int(num[0],16)>>1) & 0x01
    Rblinker   = (int(num[0],16)>>2) & 0x01
    wiper      = (int(num[0],16)>>3) & 0x01
    highbeamon = (int(num[0],16)>>5) & 0x01
    brake_a      = int(num[1],16) & 0x01
    Lblinker_a   = (int(num[1],16)>>1) & 0x01
    Rblinker_a   = (int(num[1],16)>>2) & 0x01
    wiper_a      = (int(num[1],16)>>3) & 0x01
    highbeamon_a = (int(num[1],16)>>5) & 0x01
    gyro_a = (int(num[1],16)>>6) & 0x01
    speed_a = (int(num[1],16)>>7) & 0x01
    speed = int(num[2],16)
    gyro = int(num[6],16) + (int(num[5],16)<<8)
    gyro = convert2signint(gyro,16) * (-0.00875)
    cam0   = (int(num[7],16)) & 0x01
    cam1   = (int(num[7],16)>>1) & 0x01
    cam2   = (int(num[7],16)>>2) & 0x01
    cam3   = (int(num[7],16)>>3) & 0x01
    cam4   = (int(num[7],16)>>4) & 0x01
    cam5   = (int(num[7],16)>>5) & 0x01
    cam6   = (int(num[7],16)>>6) & 0x01
    cam_set = [cam0,cam1,cam2,cam3,cam4,cam5,cam6]
    
    return [brake, Lblinker, Rblinker, wiper, highbeamon, brake_a, Lblinker_a, Rblinker_a, wiper_a, highbeamon_a, gyro_a, speed_a, speed, gyro ,cam_set]

def Q4_720_6(num):
    sign_type       = int(num[0],16) & 0xFF
    suppl_sign_type = int(num[1],16) & 0xFF
    sign_x          = int(num[2],16) & 0xFF
    sign_y          = int(num[3],16) & 0xFF
    sign_z          = int(num[4],16) & 0xFF
    filter_type     = int(num[5],16) & 0x03

    return [sign_type, suppl_sign_type, sign_x, sign_y, sign_z, filter_type]

def Q4_727(num):
    sign_type1       = int(num[0],16) & 0xFF
    suppl_sign_type1 = int(num[1],16) & 0xFF
    sign_type2       = int(num[2],16) & 0xFF
    suppl_sign_type2 = int(num[3],16) & 0xFF
    sign_type3       = int(num[4],16) & 0xFF
    suppl_sign_type3 = int(num[5],16) & 0xFF
    sign_type4       = int(num[6],16) & 0xFF
    suppl_sign_type4 = int(num[7],16) & 0xFF

    return [sign_type1, suppl_sign_type1, sign_type2, suppl_sign_type2, sign_type3, suppl_sign_type3, sign_type4, suppl_sign_type4]

def Radar_Header_505(num):
    Radar_Frame = (((int(num[5], 16)) << 8) + int(num[6], 16)) & 0xFFFF
    Func_Status = int(num[1], 16) & 0xFF
    AEB_CIPV_ID = int(num[4], 16) & 0xFF
    ACC_CIPV_ID = int(num[3], 16) & 0xFF
    CIPV_ID = int(num[2], 16) & 0xFF
    TunnelFlag = int(num[0], 16) & 0x01
    No_Obj = (int(num[0], 16) >> 2) & 0x3F

    return Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj

def Radar_VehInfo_300(num):
    YawRate_V = (int(num[1], 16) >> 3) & 0x01
    YawRate = ((int(num[2], 16) << 8) + int(num[3], 16) & 0xFFFF) * 0.1 - 100
    VehSpeed_V = (int(num[0], 16) >> 7) & 0x001
    VehSpeed = (((int(num[0], 16) << 4) + (int(num[1], 16) >> 4)) & 0xFFF) * 0.125

    return YawRate_V, YawRate, VehSpeed_V, VehSpeed

def Radar_Sta_506(num):
    Battery_Voltage_too_high = ((int(num[1], 16)) >> 3) & 0x01
    Battery_Voltage_too_low = ((int(num[1], 16)) >> 4) & 0x01
    RF2_Voltage_too_high = ((int(num[1], 16)) >> 2) & 0x01
    RF2_Voltage_too_low = ((int(num[1], 16)) >> 1) & 0x01
    RF1_Voltage_too_high = (int(num[1], 16)) & 0x01
    RF1_Voltage_too_low = ((int(num[0], 16)) >> 7) & 0x01
    MCU_Voltage_too_low = ((int(num[0], 16)) >> 6) & 0x01
    MCU_Voltage_too_high = ((int(num[0], 16)) >> 5) & 0x01
    MCU_Temp_too_low = ((int(num[0], 16)) >> 4) & 0x01
    MCU_Temp_too_high = ((int(num[0], 16)) >> 3) & 0x01
    Lost_Communication_With_Camera = ((int(num[0], 16)) >> 2) & 0x01
    Communication_Bus_Off = ((int(num[0], 16)) >> 1) & 0x01
    Radar_Sleep_Flag = (int(num[0], 16)) & 0x01

    return Battery_Voltage_too_high, Battery_Voltage_too_low, RF2_Voltage_too_high, RF2_Voltage_too_low, RF1_Voltage_too_high, RF1_Voltage_too_low, MCU_Voltage_too_low, MCU_Voltage_too_high, MCU_Temp_too_low, MCU_Temp_too_high, Lost_Communication_With_Camera, Communication_Bus_Off, Radar_Sleep_Flag

def Radar_Target_A_508(num):
    AEB_CIPVFlag = (int(num[7], 16) >> 6) & 0x01
    ACC_CIPVFlag = (int(num[7], 16) >> 7) & 0x01
    CIPVFlag = (int(num[7], 16) >> 2) & 0x01
    Vel_Y = (((int(num[4], 16) << 8) + int(num[5], 16)) & 0xFFF) * 0.05 - 102
    Vel_X = (((int(num[3], 16) << 4) + (int(num[4], 16) >> 4)) & 0xFFF) * 0.05 - 102
    Pos_Y = (((int(num[1], 16) << 8) + int(num[2], 16)) & 0xFFF) * 0.125 - 128
    Pos_X = (((int(num[0], 16) << 4) + (int(num[1], 16) >> 4)) & 0xFFF) * 0.125
    ID = int(num[6], 16) & 0xFF
    MsgCnt_A = int(num[7], 16) & 0x03

    return AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID, MsgCnt_A

def Radar_Target_B_509(num):
    Type = (int(num[7], 16) >> 2) & 0x3F
    ProbExist = int(num[5], 16) & 0x03
    DynProp = int(num[3], 16) & 0x07
    MeasStat = (int(num[3], 16) >> 5) & 0x07
    Accel_X = (((int(num[0], 16) << 4) + (int(num[1], 16) >> 4)) & 0xFFF) * 0.04 - 40
    ID = int(num[2], 16) & 0xFF
    MsgCnt_B = int(num[7], 16) & 0x03

    return Type, ProbExist, DynProp, MeasStat, Accel_X, ID, MsgCnt_B

def sort_key(file_name):
    return file_name.split('_')[-1]


bagpath = './'
bag_coordinates = []
bag_files = [f for f in os.listdir(bagpath) if f.endswith(".bag")]
sorted_bag_files = sorted(bag_files, key=sort_key)

for files in sorted_bag_files:
    if files[-4:] == ".bag":
        print(f"parsing\t{files} ......")
        bag_file = os.path.join(bagpath, files)
        bag_name = files.split('.')[0]

        # Output dir
        output_path     = os.path.join(bagpath, bag_name)
        timestamp_path  = os.path.join(output_path, "timestamp")
        g5_path         = os.path.join(output_path, "IVA_g5")
        q4_path         = os.path.join(output_path, "Mobileye_q4")
        image_path      = os.path.join(output_path, "image")
        imu_path        = os.path.join(output_path, "imu")
        vls128_path     = os.path.join(output_path, "VLS128_pcdnpy")
        m1lidar_path    = os.path.join(output_path, "m1lidar_pcdnpy")
        radar_path      = os.path.join(output_path, "radar")

        # 這兩個一定會有
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        if not os.path.exists(timestamp_path):
            os.makedirs(timestamp_path)

        # OPEN BAG
        try:
            bag = rosbag.Bag(bag_file, "r")
        except rosbag.bag.ROSBagException as e:
            print(f"Error opening ROS Bag file: {e}")
            continue

        topic_to_read = ["/can0/received_msg", "/can2/received_msg", "/cme_cam", "/imu", "/velodyne_points", "/rslidar_points"]
        data_name = ["IVAG5", "MobQ4", "Image", "Imu", "VLS128", "M1Lidar", "Radar"]
        data_dict = {
            data: [] for data in data_name
        }
        # data_dict 包含了每個 topic 對應的時間戳列表

        lane_ok = 0
        object_ok = False
        header_id = None
        first_header = None

        messages = bag.read_messages(topics=topic_to_read)
        for topic, msg, t in messages:
            
            timefloat = msg.header.stamp.to_sec()
            timestr = "{:.6f}".format(timefloat)
            
            if (topic == "/can0/received_msg"):
                id = hex(msg.id)
                # 只有0x4e1一定會存在, 其他的id不一定存在, 所以要透過前後id, 猜g5的開頭id在哪裡

                # 沒有object訊息(0x4e7 0x4e8)用的 想不到好的邏輯
                if (id == "0x4e1"):
                    header_id = timefloat
                if (header_id is not None and header_id < timefloat):
                    if (id in {"0x4e2", "0x4e9", "0x4e3", "0x4ea", "0x4e4", "0x4eb", "0x4e5", "0x4ec", "0x4e6"}):
                        lane_ok += 1
                        if (lane_ok == 1):
                            data_dict["IVAG5"].append(header_id)
                else:
                    lane_ok = 0
                if (id in {"0x4e7", "0x4e8"}):
                    pass

                ###radar###
                if (id == "0x505"):
                    data_dict["Radar"].append(timefloat)
                    
            elif (topic == "/can2/received_msg"):
                if (hex(msg.id) == "0x700"):
                    data_dict["MobQ4"].append(timefloat)

            elif (topic == "/cme_cam"):
                data_dict["Image"].append(timefloat)
                
            elif (topic == "/imu"):
                data_dict["Imu"].append(timefloat)
                
            elif (topic == "/velodyne_points"):
                data_dict["VLS128"].append(timefloat)

            elif (topic == "/rslidar_points"):
                data_dict["M1Lidar"].append(timefloat)
        
        bag.close()



        # 從 data_dict 挑出想要的 timestamp 存到 time_data_arranged
        time_g5_current = None
        time_q4_current = None
        time_image_current = None
        time_imu_current = None
        time_m1lidar_current = None
        time_vls128_current = None
        time_radar_current = None
        time_data_arranged = []
        data_list = [] # list -> tuple
        dataCount = 0


        avaliabale_sensor = []
        # 檢查哪一個設備沒收到
        for key, value in data_dict.items():
            if (not value):
                print(f"{key} not found")
            else:
                avaliabale_sensor.append(key)
        

        for time_g5_index in range(len(data_dict["IVAG5"])):
            
            data_list.clear()
            data_list.append(data_dict["IVAG5"][time_g5_index])

            valid = True
            complete = True

            for time_q4_index in range (len(data_dict["MobQ4"])):

                # Q4 找離 G5 後最近的
                if (data_dict["IVAG5"][time_g5_index] < data_dict["MobQ4"][time_q4_index]):
                    
                    if (time_q4_current == None):
                        time_q4_current = time_q4_index
                        data_list.append(data_dict["MobQ4"][time_q4_current])
            if (time_q4_current == None and "ModQ4" in avaliabale_sensor):
                data_list.append(None)

            for time_image_index in range (len(data_dict["Image"])):
                
                # Image 找離 G5 後最近的
                if (data_dict["IVAG5"][time_g5_index] < data_dict["Image"][time_image_index]):

                    if (time_image_current == None):
                        time_image_current = time_image_index
                        data_list.append(data_dict["Image"][time_image_index])
            if (time_image_current == None and "Image" in avaliabale_sensor):
                data_list.append(None)

            for time_imu_index in range (len(data_dict["Imu"])):

                # Imu 找離 G5 後最近的
                if (data_dict["IVAG5"][time_g5_index] < data_dict["Imu"][time_imu_index]):
                    
                    if (time_imu_current == None):
                        time_imu_current = time_imu_index
                        data_list.append(data_dict["Imu"][time_imu_index])
            if (time_imu_current == None and "Imu" in avaliabale_sensor):
                data_list.append(None)

            for time_vls128_index in range (len(data_dict["VLS128"])):

                # VLS128 找離 Image 前(大概)0.2秒
                if ((time_image_current is not None) and
                    (0.15 < (data_dict["Image"][time_image_current] - data_dict["VLS128"][time_vls128_index]) < 0.25)):
                    if (time_vls128_current == None):
                        time_vls128_current = time_vls128_index
                        data_list.append(data_dict["VLS128"][time_vls128_index])
            if (time_vls128_current == None and "VLS128" in avaliabale_sensor):
                data_list.append(None)

            for time_m1lidar_index in range (len(data_dict["M1Lidar"])):              
                
                # M1Lidar 找離 Image 前(大概)0.23秒
                if ((time_image_current is not None) and 
                    (0.18 < (data_dict["Image"][time_image_current] - data_dict["M1Lidar"][time_m1lidar_index]) < 0.28)):
                    if (time_m1lidar_current == None):
                        time_m1lidar_current = time_m1lidar_index
                        data_list.append(data_dict["M1Lidar"][time_m1lidar_index])
            if (time_m1lidar_current == None and "M1Lidar" in avaliabale_sensor):
                data_list.append(None)

            for time_radar_index in range (len(data_dict["Radar"])):

                # Radar 找離 G5 後最近的
                if (data_dict["IVAG5"][time_g5_index] < data_dict["Radar"][time_radar_index]):
                    if (time_radar_current == None):
                        time_radar_current = time_radar_index
                        data_list.append(data_dict["Radar"][time_radar_index])
            if (time_radar_current == None and "Radar" in avaliabale_sensor):
                data_list.append(None)

            # list -> tuple
            data_tuple = tuple(data_list)

            # 忽略第一筆資料
            if (dataCount != 0):
                # 找看看1個set中有沒有一樣的timestamp
                for i in range(len(data_tuple)):
                    if (time_data_arranged[dataCount-1][i] == data_tuple[i]):
                        valid = False
            # 時間對齊上有資料缺失
            if (sum(1 for value in data_tuple if value is not None) != len(avaliabale_sensor)):
                complete = False

            """test arranged time stamp output"""
            print(f"No. {dataCount:06d}")
            print(f"Valid\t\t => {valid}")
            print(f"Complete\t => {complete}")
            for data in range(len(data_tuple)):
                if (data_tuple[data] is None):
                    print(f"{avaliabale_sensor[data]} time\t => None")
                else:
                    print(f"{avaliabale_sensor[data]} time\t => {data_tuple[data]:.6f}")
            print("--------------------------------------------------------------------")

            if (valid and complete):
                dataCount += 1
                time_data_arranged.append(data_tuple)

            time_q4_current = time_image_current = time_imu_current = time_m1lidar_current = time_vls128_current = time_radar_current = None
        # breakpoint()




        # 從 bag 讀取 time_data_arranged timestamp 的資料 順便把最後一筆資料濾掉
        for data_tuples in range (len(time_data_arranged) - 1):
            # Flags
            g5_start = False
            q4_start = False
            q4_obj_start = False
            imu_start = False
            radar_start = False

            Mobileye_Q4_line_type = ["Undecided", "Solid", "Dahsed", "Double Line Marking", "Basis of Track for Temporary Shifts", "Deceleration", "High Occupancy Vehicle Lane"]
            Mobileye_Q4_object_class_name = ["car", "truck", "bike", "bicycle", "pedestrian", "general_object", "unknown", "uncertain_vcl"]
            IVA_G5_line_type = ["Undecided", "Single Solid", "Double Solid", "Single Dashed", "Road Edge"]
            IVA_G5_line_color = ["Undecided", "White", "Yellow", "Red", "Blue", "Pink"]
            IVA_G5_lanemark = ["Undecided", "本車道", "左車道", "右車道"]
            IVA_G5_object_class_name = ["background", "car_big", "car_small", "motor", "bike", "person", "e_motor_pd", "e_bike_pd", "e_car_big", "e_car_small", "e_motor", "e_bike"]
            radar_target_type = ["Unknown", "Pedestrian", "Bike", "Car", "Truck"]
            radar_ProbExist = ["25%", "50%", "75%", "99%"]
            radar_DynProb = ["Unclassified", "Standing", "Stopped", "Moving", "Oncoming", "Flyover", "Ditchover"]
            radar_MeasStat = ["No object", "New Object", "Predicted Object", "Measured Object", "Invalid"]

            g5_l1, g5_l1_info, g5_r1, g5_r1_info, g5_l2, g5_l2_info, g5_r2, g5_r2_info, g5_lane_overview, g5_obj_overview, g5_obj = (
                [] for _ in range (11)
            )
            q4_l1_info, q4_l1, q4_r1_info, q4_r1, q4_lr_info, q4_lr, q4_rl_info, q4_rl, q4_ll_info, q4_ll, q4_rr_info, q4_rr, q4_obj_overview, q4_obj, q4_obj_pos, q4_obj_other = (
                [] for _ in range (16)
            )
            radar_header, radar_targetA, radar_targetB = (
                [] for _ in range (3)
            )

            # OPEN BAG
            try:
                bag = rosbag.Bag(bag_file, "r")
            except rosbag.bag.ROSBagException as e:
                print(f"Error opening ROS Bag file: {e}")
                continue
            
            # print("====================================================================")

            ######IVA G5######
            if ("IVAG5" in avaliabale_sensor):
                index = avaliabale_sensor.index("IVAG5")
                messages = bag.read_messages(topics="/can0/received_msg")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        g5_start = True

                        timestamp_output = os.path.join(timestamp_path, "IVA_g5.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        # print(topic, timestr)
                        # print(f"IVA G5 Start => {timestr}")

                    if (g5_start):
                        if ((hex(msg.id) == "0x4e1") and ((timefloat - time_data_arranged[data_tuples][index]) > 0.075)):
                            
                            g5_start = False

                            if not os.path.exists(g5_path):
                                os.makedirs(g5_path)

                            out_file = os.path.join(g5_path, f"{data_tuples:06d}.txt")
                            f = open(out_file, 'w')
                            print(g5_l1, file=f)
                            print(g5_l1_info, file=f)
                            print(g5_r1, file=f)
                            print(g5_r1_info, file=f)
                            print(g5_l2, file=f)
                            print(g5_l2_info, file=f)
                            print(g5_r2, file=f)
                            print(g5_r2_info, file=f)
                            print(g5_lane_overview, file=f)
                            print(g5_obj_overview, file=f)
                            print(','.join(map(str, g5_obj)), file=f)
                            f.close()

                            # print(f"IVA G5 Stop => {timestr}")

                        else:
                            data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

                            # Lane Host Left
                            if (hex(msg.id) == "0x4e2"):
                                quality, exist, c0, c1, c2, c3 = IVA_lanecurve_2901379(data_str, flag='1')
                                g5_l1 = f"{quality},{exist},{c0},{c1},{c2},{c3}"

                            elif (hex(msg.id) == "0x4e9"):
                                fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVA_lanecurve_2902468(data_str)
                                g5_l1_info = f"{fit_score_far},{fit_score_mid},{fit_score_close},{distance_score},{detect_score}"
                            
                            # Lane Host Right
                            elif (hex(msg.id) == "0x4e3"):
                                quality, exist, c0, c1, c2, c3 = IVA_lanecurve_2901379(data_str, flag='3')
                                g5_r1 = f"{quality},{exist},{c0},{c1},{c2},{c3}"

                            elif (hex(msg.id) == "0x4ea"):
                                fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVA_lanecurve_2902468(data_str)
                                g5_r1_info = f"{fit_score_far},{fit_score_mid},{fit_score_close},{distance_score},{detect_score}"
                            
                            # Lane Adjacent Left
                            elif (hex(msg.id) == "0x4e4"):
                                quality, exist, c0, c1, c2, c3 = IVA_lanecurve_2901379(data_str, flag='7')
                                g5_l2 = f"{quality},{exist},{c0},{c1},{c2},{c3}"

                            elif (hex(msg.id) == "0x4eb"):
                                fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVA_lanecurve_2902468(data_str)
                                g5_l2_info = f"{fit_score_far},{fit_score_mid},{fit_score_close},{distance_score},{detect_score}"
                            
                            # Lane Adjacent Right
                            elif (hex(msg.id) == "0x4e5"):
                                quality, exist, c0, c1, c2, c3 = IVA_lanecurve_2901379(data_str, flag='9')
                                g5_r2 = f"{quality},{exist},{c0},{c1},{c2},{c3}"

                            elif (hex(msg.id) == "0x4ec"):
                                fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVA_lanecurve_2902468(data_str)
                                g5_r2_info = f"{fit_score_far},{fit_score_mid},{fit_score_close},{distance_score},{detect_score}"
                            
                            # Lane Overview
                            elif (hex(msg.id) == "0x4e6"):
                                dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, color_L_1st, color_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd, color_L_2nd, color_R_2nd = IVA_Lane_Distance_decode(data_str)
                                g5_lane_overview = f"{dis_L_1st},{dis_R_1st},{type_L_1st},{type_R_1st},{color_L_1st},{color_R_1st},{dis_L_2nd},{dis_R_2nd},{type_L_2nd},{type_R_2nd},{color_L_2nd},{color_R_2nd}"

                            # Object Overview
                            elif (hex(msg.id) == "0x4e8"):
                                G5obnum, G5lm = IVA_obj_290D(data_str)
                                g5_obj_overview = f"{G5obnum},{','.join(map(str, G5lm))}"

                            elif (hex(msg.id) == "0x4e7"):
                                OBJ_id, OBJ_class, OBJ_conf, OBJ_pos_x, OBJ_pos_y, OBJ_vel_x, OBJ_vel_y = IVA_obj_290B(data_str)
                                g5_obj.extend([OBJ_id, OBJ_class, OBJ_conf, OBJ_pos_x, OBJ_pos_y, OBJ_vel_x, OBJ_vel_y])
            
            ######Mobileye Q4######
            if ("MobQ4" in avaliabale_sensor):
                index = avaliabale_sensor.index("MobQ4")
                messages = bag.read_messages(topics="/can2/received_msg")

                for topic, msg, t in messages:
                    
                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        q4_start = True
                        q4_obj_start = True

                        timestamp_output = os.path.join(timestamp_path, "Mobileye_q4.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        # print(topic, timestr)
                        # print(f"Mobileye Q4 Start => {timestr}")

                    if (q4_start):
                        if (hex(msg.id) == "0x700" and ((timefloat - time_data_arranged[data_tuples][index]) > 0)):
                            q4_start = False
                            q4_obj_start = False

                            if not os.path.exists(q4_path):
                                os.makedirs(q4_path)

                            out_file = os.path.join(q4_path, f"{data_tuples:06d}.txt")
                            f = open(out_file, 'w')
                            print(','.join(map(str, q4_l1_info)), file=f)
                            print(','.join(map(str, q4_l1)), file=f)
                            print(','.join(map(str, q4_r1_info)), file=f)
                            print(','.join(map(str, q4_r1)), file=f)
                            print(','.join(map(str, q4_lr_info)), file=f)
                            print(','.join(map(str, q4_lr)), file=f)
                            print(','.join(map(str, q4_rl_info)), file=f)
                            print(','.join(map(str, q4_rl)), file=f)
                            print(','.join(map(str, q4_ll_info)), file=f)
                            print(','.join(map(str, q4_ll)), file=f)
                            print(','.join(map(str, q4_rr_info)), file=f)
                            print(','.join(map(str, q4_rr)), file=f)
                            print(','.join(map(str, q4_obj_overview)), file=f)
                            print(','.join(map(str, q4_obj)), file=f)
                            print(','.join(map(str, q4_obj_pos)), file=f)
                            print(','.join(map(str, q4_obj_other)), file=f)

                            f.close()

                            # print(f"Mobileye Q4 Stop => {timestr}")

                        else:
                            data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

                            if (hex(msg.id) == "0x770"):
                                # 主車道第一筆資料
                                Q4L_conf, Q4L_type, Q4L_side, Q4L_start, Q4L_end = Q4_770_772(data_str)
                                q4_l1_info = Q4L_conf, Q4L_type, Q4L_side, Q4L_start, Q4L_end

                            elif (hex(msg.id) == "0x771"):
                                q4_l1 = Q4_771_773(data_str)

                            elif (hex(msg.id) == "0x772"):
                                Q4R_conf, Q4R_type, Q4R_side, Q4R_start, Q4R_end = Q4_770_772(data_str)
                                q4_r1_info = Q4R_conf, Q4R_type, Q4R_side, Q4R_start, Q4R_end
                            
                            elif (hex(msg.id) == "0x773"):
                                q4_r1 = Q4_771_773(data_str)

                            elif (hex(msg.id) == "0x784"):
                                # 相鄰車道第一筆資料
                                adj_type, adj_conf, adj_start, adj_end, adj_role = Q4_78468A(data_str)
                                q4_lr_info = adj_type, adj_conf, adj_start, adj_end, adj_role

                            elif (hex(msg.id) == "0x785"):
                                q4_lr = Q4_78579B(data_str)
                                f_adj=1

                            elif (hex(msg.id) == "0x786"):
                                adj_type1, adj_conf1, adj_start1, adj_end1, adj_role1 = Q4_78468A(data_str)
                                q4_rl_info = adj_type1, adj_conf1, adj_start1, adj_end1, adj_role1

                            elif (hex(msg.id) == "0x787"):
                                q4_rl = Q4_78579B(data_str)
                                f_adj1=1

                            elif (hex(msg.id) == "0x788"):
                                adj_typ2, adj_conf2, adj_start2, adj_end2, adj_role2 = Q4_78468A(data_str)
                                q4_ll_info = adj_typ2, adj_conf2, adj_start2, adj_end2, adj_role2

                            elif (hex(msg.id) == "0x789"):
                                q4_ll = Q4_78579B(data_str)
                                f_adj2=1

                            elif (hex(msg.id) == "0x78a"):
                                adj_typ3, adj_conf3, adj_start3, adj_end3, adj_role3 = Q4_78468A(data_str)
                                q4_rr_info = adj_typ3, adj_conf3, adj_start3, adj_end3, adj_role3

                            elif (hex(msg.id) == "0x78b"):
                                q4_rr = Q4_78579B(data_str)
                                f_adj3=1

                            elif (hex(msg.id) == "0x541"):
                                # "CIPV" 通常指的是 "Critical Instant of Possible Collision Vehicle"，也就是可能發生碰撞的關鍵瞬間車輛。
                                Mob_Num_Objs, CIPV, CIPVlost, VDacc = Q4_541(data_str)
                                q4_obj_overview = Mob_Num_Objs, CIPV, CIPVlost, VDacc

                            elif (hex(msg.id) in ["0x500", "0x503", "0x506", "0x509", "0x50c", "0x50f", "0x512", "0x515", "0x518", "0x51b", "0x51e", "0x521"]):
                                iddd, Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, OBJ_Lane_Ass, Relative_Lat_Velocity = Q4_500(data_str)
                                if (iddd != 0):
                                    q4_obj_start = True
                                    q4_obj.extend((iddd, Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, OBJ_Lane_Ass, Relative_Lat_Velocity))
                                else:
                                    q4_obj_start = False

                            elif (hex(msg.id) in ["0x501", "0x504", "0x507", "0x50a", "0x50d", "0x50g", "0x510", "0x513", "0x516", "0x519", "0x51c", "0x51f", "0x522"]):
                                Absolute_Long_Acc, Long_Distance, Lateral_Distance = Q4_501(data_str)
                                if (q4_obj_start):
                                    q4_obj_pos.extend((Absolute_Long_Acc, Long_Distance, Lateral_Distance))
                                
                            elif (hex(msg.id) in ["0x502", "0x505", "0x508", "0x50b", "0x50e", "0x511", "0x514", "0x517", "0x51a", "0x51d", "0x520", "0x523"]):
                                Absolute_Speed, OBJ_Motion_Status, OBJ_Motion_Category, Brake_Light, Turn_Indicator_Right, Turn_Indicator_Left, Light_indicator_validity, OBJ_Angle_Mid, OBJ_Angle_Rate = Q4_502(data_str)
                                if (q4_obj_start):
                                    q4_obj_other.extend((Absolute_Speed, OBJ_Angle_Mid, OBJ_Angle_Rate))

                            elif (hex(msg.id) in ["0x720", "0x721", "0x722", "0x723", "0x724", "0x725", "0x726"]):
                                Mob_tsr0 = Q4_720_6(data_str)

                            elif (hex(msg.id) == "0x727"):
                                Mob_Sign_type = Q4_727(data_str)

            ######Image######
            if ("Image" in avaliabale_sensor):
                index = avaliabale_sensor.index("Image")
                messages = bag.read_messages(topics="/cme_cam")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        timestamp_output = os.path.join(timestamp_path, "image.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        # print(topic, timestr)
                        # print(f"Image Start => {timestr}")

                        img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

                        # OUTPUT Image
                        if not os.path.exists(image_path):
                            os.makedirs(image_path)

                        image_output = os.path.join(image_path, f"{data_tuples:06d}.png")
                        cv2.imwrite(image_output, img)

            ######Imu######
            if ("Imu" in avaliabale_sensor):
                index = avaliabale_sensor.index("Imu")
                messages = bag.read_messages(topics="/imu")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        imu_start = True

                        # print(topic, timestr)
                        # print(f"Imu Start => {timestr}")
                    
                    if (imu_start):
                        if (timefloat == time_data_arranged[data_tuples + 1][index]):
                            imu_start = False

                        else:
                            timestamp_output = os.path.join(timestamp_path, "imu.txt")
                            time_f = open(timestamp_output, 'a+')
                            print(timefloat, file=time_f)
                            time_f.close()

                            if not os.path.exists(imu_path):
                                os.makedirs(imu_path)

                            out_file = os.path.join(imu_path, f"{data_tuples:06d}.txt")
                            f = open(out_file, 'a+')
                            print(f"{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}", file=f)
                            f.close()


            ######M1Lidar######
            if ("M1Lidar" in avaliabale_sensor):
                index = avaliabale_sensor.index("M1Lidar")
                messages = bag.read_messages(topics="/rslidar_points")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        timestamp_output = os.path.join(timestamp_path, "m1lidar.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        # print(topic, timestr)
                        # print(f"M1Lidar Start => {timestr}")
                        
                        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                        points = np.array(list(gen))
                        points_xyz = points[:, :3]
                        points_other = points[:, 3:]

                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points_xyz)

                        if not os.path.exists(m1lidar_path):
                            os.makedirs(m1lidar_path)

                        out_file = os.path.join(m1lidar_path, f"{data_tuples:06d}.npy")
                        np.save(out_file, points)
                        out_file = os.path.join(m1lidar_path, f"{data_tuples:06d}.pcd")
                        o3d.io.write_point_cloud(out_file, pcd)
            
            ######VLS128######
            if ("VLS128" in avaliabale_sensor):
                index = avaliabale_sensor.index("VLS128")
                messages = bag.read_messages(topics="/velodyne_points")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)
                    
                    if (timefloat == time_data_arranged[data_tuples][index]):

                        timestamp_output = os.path.join(timestamp_path, "VLS128.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        # print(topic, timestr)
                        # print(f"M1Lidar Start => {timestr}")
                        
                        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)
                        points = np.array(list(gen))
                        points_xyz = points[:, :3]
                        points_other = points[:, 3:]

                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points_xyz)

                        if not os.path.exists(vls128_path):
                            os.makedirs(vls128_path)

                        out_file = os.path.join(vls128_path, f"{data_tuples:06d}.npy")
                        np.save(out_file, points_other)
                        out_file = os.path.join(vls128_path, f"{data_tuples:06d}.pcd")
                        o3d.io.write_point_cloud(out_file, pcd)

            ######Radar######
            if ("Radar" in avaliabale_sensor):
                index = avaliabale_sensor.index("Radar")
                messages = bag.read_messages(topics="/can0/received_msg")

                for topic, msg, t in messages:

                    timefloat = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timefloat)

                    if (timefloat == time_data_arranged[data_tuples][index]):

                        radar_start = True

                        timestamp_output = os.path.join(timestamp_path, "radar.txt")
                        time_f = open(timestamp_output, 'a+')
                        print(timefloat, file=time_f)
                        time_f.close()

                        if not os.path.exists(radar_path):
                            os.makedirs(radar_path)

                        out_file = os.path.join(radar_path, f"{data_tuples:06d}.txt")
                        f = open(out_file, 'w')

                        # print(topic, timestr)
                        # print(f"Radar Start => {timestr}")

                    if (radar_start):
                        if (hex(msg.id) == "0x505" and ((timefloat - time_data_arranged[data_tuples][index]) > 0)):
                            radar_start = False

                            f.close()

                            # print(f"Radar Stop => {timestr}")

                        else:
                            data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

                            if (hex(msg.id) == "0x505"):
                                Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj = Radar_Header_505(data_str)
                                radar_header = Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj

                                print(','.join(map(str, radar_header)), file=f)

                            elif (hex(msg.id) in {"0x508", "0x50a", "0x50c", "0x50e", "0x510", "0x512", "0x514", "0x516", "0x518", "0x51a", "0x51c", "0x51e", "0x520", "0x522", "0x524", "0x526", "0x528", "0x52a", "0x52c", "0x52e", "0x530", "0x532", "0x534", "0x536", "0x538", "0x53a", "0x53c", "0x53e", "0x540", "0x542", "0x544", "0x546"}):
                                AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID_A, MsgCnt_A = Radar_Target_A_508(data_str)
                                radar_targetA = AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID_A, MsgCnt_A

                            elif (hex(msg.id) in {"0x509", "0x50b", "0x50d", "0x50f", "0x511", "0x513", "0x515", "0x517", "0x519", "0x51b", "0x51d", "0x51f", "0x521", "0x523", "0x525", "0x527", "0x529", "0x52b", "0x52d", "0x52f", "0x531", "0x533", "0x535", "0x537", "0x539", "0x53b", "0x53d", "0x53f", "0x541", "0x543", "0x545", "0x547"}):
                                Type, ProbExist, DynProp, MeasStat, Accel_X, ID_B, MsgCnt_B = Radar_Target_B_509(data_str)
                                radar_targetB = Type, ProbExist, DynProp, MeasStat, Accel_X, ID_B, MsgCnt_B
                                
                                print(','.join(map(str, radar_targetA[0:8] + radar_targetB[0:5])), file=f)

            # CLOSE BAG
            bag.close()


        # # Move Bag into file
        # shutil.move(bag_file, output_path)

        print(f"Finished\t{files} !!!!!!!")

        ######GPS######
        with rosbag.Bag(bag_file, 'r') as bag:
            out_file = os.path.join(output_path, "gps.txt")

            for toipc, msg, t in bag.read_messages(topics=["/fix"]):

                f = open(out_file, '+a')
                print(msg.latitude, msg.longitude, file=f)
                if (f"{msg.latitude}" != "nan"):
                    bag_coordinates.append({
                        "coordinates": [msg.latitude, msg.longitude],
                        "bag_name": bag_name
                    })
            f.close()


######Map######
if (bag_coordinates):
    taiwan_map = folium.Map(location=bag_coordinates[0]["coordinates"], zoom_start=16, tiles="OpenStreetMap")

    for c in bag_coordinates:
        folium.Marker(
            location=c["coordinates"],
            popup=c["bag_name"],
            icon=folium.Icon(
                color="black",
                icon_color="red",
                icon="car",
                prefix="fa"
            )
        ).add_to(taiwan_map)

        folium.PolyLine(
            locations=[c["coordinates"] for c in bag_coordinates],
            # tooltip=c["bag_name"],
            # popup="line",
            color="blue",
            weight=3
        ).add_to(taiwan_map)

    # 顯示地圖 以父目錄名稱命名
    parent_dir_name = os.path.basename(os.path.dirname(os.path.realpath(__file__)))
    taiwan_map.save(f"{parent_dir_name}.html")

    # subprocess.call(["google-chrome", "taiwan_map.html"])

    print("Map DONE !")