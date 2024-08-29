import numpy as np
import os
import rosbag
import cv2
import shutil
import folium
import subprocess
import datetime
import argparse
from tqdm import tqdm
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
import open3d as o3d



class RosbagUtils:
    @staticmethod
    def prettyhex(nums, sep=''):
        return sep.join(f'{a:02x}' for a in nums)
    
    @staticmethod
    def prettyhex2(dlc, nums, sep=''):
        return sep.join(f'{a:02x}' for a in nums[0:dlc])
    
    @staticmethod
    def convert2signint(value, bits):
        if value & (1 << (bits - 1)):
            value -= (1 << bits)
        return value
    
    @staticmethod
    def get_curve(_C_L,_C_R,L_max_dis,R_max_dis):
        Y_L =  np.linspace(0,L_max_dis,720) # distance
        Y_R =  np.linspace(0,R_max_dis,720) # distance
        X_R, X_L =0,0
        for i in range(4):
            X_L = X_L + (_C_L[i]*(Y_L**i))
            X_R = X_R + (_C_R[i]*(Y_R**i))
        return X_L, X_R, Y_L, Y_R

    @staticmethod
    def get_curve1(_C_R,R_max_dis):
        Y_R =  np.linspace(0,R_max_dis,720) # distance
        X_R, X_L = 0, 0
        for i in range(4):
            X_R = X_R + (_C_R[i]*(Y_R**i))
        return X_R, Y_R
    
    @staticmethod
    def point2IPM (xs, ys):
        point_X, point_Y = [], []
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
            out_y = Camera_Height * (Ky * cos_theta - (y - (input_image_height/2)) * sin_theta) / (Ky * sin_theta + (y - (input_image_height >> 1)) * cos_theta)
            out_x = (x - (input_image_width/2)) * (out_y * cos_theta - Camera_Height_sin_theta) / Kx
            point_X.append(out_x)
            point_Y.append(out_y)
        return point_X, point_Y

    @staticmethod
    def IPM2point (xs, ys):
        point_X, point_Y = [], []
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
        return point_X, point_Y

    @staticmethod
    def get_two_float(f_str, n):
        f_str = str(f_str)
        a, b, c = f_str.partition('.')
        c = (c+"0"*n)[:n]
        return ".".join([a, c])
    
    @staticmethod
    def rot(a,theta):
        return a[1]*np.sin(theta)+a[0]*np.cos(theta), a[1]*np.cos(theta)-a[0]*np.sin(theta)

    
class IVAG5MessageParser:
    @staticmethod
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
        return ID, classes, conf, X, Y, vx ,vy

    @staticmethod
    def IVA_obj_290D(num):
        objnum = int(num[0],16) & 0xFF
        lms=[]

        for i in range(5):
            lm0 = int(num[i],16) & 0x03
            lm1 = (int(num[i],16)>>2) & 0x03
            lm2 = (int(num[i],16)>>4) & 0x03
            lm3 = (int(num[i],16)>>6) & 0x03
            lms.extend([lm0, lm1, lm2, lm3])
        return objnum, lms

    @staticmethod
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
        return dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, color_L_1st, color_R_1st, \
               dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd, color_L_2nd, color_R_2nd

    @staticmethod
    def IVA_lanecurve_2902468(num):
        # type and color
        fit_score_far = int(num[0],16)
        fit_score_mid = int(num[1],16)
        fit_score_close = int(num[2],16)
        distance_score = int(num[3],16)
        detect_score = int(num[4],16)
        return  fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score

    @staticmethod
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
        return quality, exist, c0, c1, c2, c3

    @staticmethod
    def IVA_obj_num_decode(num):
        number = num[0]
        return [number]


class MobQ4MessageParser:
    @staticmethod
    def Q4_541(num):
        objnum = int(num[1],16) & 0x0F
        CIPV = ((int(num[1],16)>>4) + (int(num[2],16)<<4)) & 0x7F
        CIPVlost = (int(num[2],16)>>3) & 0x03
        acc = (int(num[2],16)>>5) & 0x03
        return objnum, CIPV, CIPVlost, acc

    @staticmethod
    def Q4_500(num):
        ID = int(num[0],16) & 0x7F
        cls = ((int(num[0],16)>>7) + (int(num[1],16)<<1)) & 0x07
        w = (((int(num[1],16)>>2) + (int(num[2],16)<<6)) & 0x7F) * 0.05
        l = (((int(num[2],16)>>1) + (int(num[3],16)<<7)) & 0x1FF) * 0.05
        relative_long_v = (((int(num[3],16)>>2) + (int(num[4],16)<<6)) & 0x1FFF) * 0.05 - 120
        obj_lane_assignment = ((int(num[4],16)>>7) + (int(num[5],16)<<1)) & 0x07
        relative_lat_v = (((int(num[5],16)>>2) + (int(num[6],16)<<6)) & 0x7FF) * 0.05 - 20
        return ID, cls, w, l, relative_long_v, obj_lane_assignment, relative_lat_v

    @staticmethod
    def Q4_501(num):
        abs_long_acc = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x1FF) * 0.05 - 12.8
        long_dis = (((int(num[1],16)>>1) + (int(num[2],16)<<7)) & 0x1FFF) * 0.05
        lat_dis = (((int(num[2],16)>>6) + (int(num[3],16)<<2) + (int(num[4],16)<<10)) & 0xFFF) * 0.05 - 102.4
        # long_dis_M = (((int(num[4],16)>>2) + (int(num[5],16)<<6)) & 0x1FFF) * 0.05    #bag error
        return abs_long_acc, long_dis, lat_dis

    @staticmethod
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
        return abs_speed, ms_status, ms_cate, brake, turnR, turnL, light_valid, obj_angle_mid, obj_angle_rate

    @staticmethod
    def Q4_76E(num):
        return int(num[0],16) & 0x01

    @staticmethod
    def Q4_770_772(num):
        LH_conf = int(num[0],16) & 0x7F
        LH_landmark_type = ((int(num[0],16)>>7) + (int(num[1],16)<<1)) & 0x0F
        LH_side = (int(num[1],16) >>3) & 0x03
        LH_VR_start = (((int(num[1],16)>>5) + (int(num[2],16)<<3) + (int(num[3],16)<<11)) & 0x7FFF) * 0.01
        LH_VR_end = (((int(num[3],16)>>4) + (int(num[4],16)<<4) + (int(num[5],16)<<12)) & 0x7FFF) * 0.01
        return LH_conf, LH_landmark_type, LH_side, LH_VR_start, LH_VR_end

    @staticmethod
    def Q4_771_773(num):
        c0 = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x7FF) * 0.01 - 10
        c1 = (((int(num[1],16)>>3) + (int(num[2],16)<<5)) & 0x3FF) * 0.000977 - 0.357
        c2 = (((int(num[2],16)>>5) + (int(num[3],16)<<3) + (int(num[4],16)<<11)) & 0xFFFF) * 0.000000977 - 0.032
        c3 = (((int(num[4],16)>>5) + (int(num[5],16)<<3) + (int(num[6],16)<<11)) & 0xFFFF) * 0.00000000373 - 0.000122
        return c0, c1, c2, c3

    @staticmethod
    def Q4_782(num):
        lane_adj_count = (int(num[0],16)>>4) & 0x0F
        return lane_adj_count

    @staticmethod
    def Q4_78468A(num):
        typ = int(num[0],16) & 0x0F
        conf = (((int(num[0],16)>>4) + (int(num[1],16)<<4)) & 0x7F) * 0.01
        VR_start = (((int(num[1],16)>>3) + (int(num[2],16)<<5) + (int(num[3],16)<<13)) & 0x7FFF) * 0.01
        VR_end = (((int(num[3],16)>>2) + (int(num[4],16)<<6) + (int(num[5],16)<<14)) & 0x7FFF) * 0.01
        line_role = (int(num[5],16)>>1) & 0x0F
        return typ, conf, VR_start, VR_end, line_role

    @staticmethod
    def Q4_78579B(num):
        c0 = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x7FF) * 0.01 - 10
        c1 = (((int(num[1],16)>>3) + (int(num[2],16)<<5)) & 0x3FF) * 0.000977 - 0.357
        c2 = (((int(num[2],16)>>5) + (int(num[3],16)<<3) + (int(num[4],16)<<11)) & 0xFFFF) * 0.000000977 - 0.032
        c3 = (((int(num[4],16)>>5) + (int(num[5],16)<<3) + (int(num[6],16)<<11)) & 0xFFFF) * 0.00000000373 - 0.000122
        return c0, c1, c2, c3

    @staticmethod
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
        return ver, frameid, freesight, rain, fog, splash, sunray, lowsun, blurimg, partial_blockage, \
               full_blockage, frozen_blockage, outof_calib, outof_focus, TSR_outof_focus

    @staticmethod
    def Q4_486(num):
        img_timestamp = (int(num[1],16) + (int(num[2],16)<<8) + (int(num[3],16)<<16) + (int(num[4],16)<<24)) & 0xFFFFFFFF
        hzd_objnum = int(num[5],16) & 0x7F
        valid_frame = (int(num[5],16)<<6) & 0x01
        return img_timestamp, hzd_objnum, valid_frame

    @staticmethod
    def Q4_487(num): 
        UTC_timestamp = int(num[0],16) + (int(num[1],16)<<8) + (int(num[1],16)<<16) + (int(num[3],16)<<24) + (int(num[4],16)<<32) + (int(num[5],16)<<40) + (int(num[6],16)<<48)
        return UTC_timestamp

    @staticmethod
    def Q4_488ACE(num):
        hzd_id   = int(num[0],16) & 0xFF
        hzd_prob = ((int(num[1],16) + (int(num[2],16)<<8)) & 0x3FF)
        hzd_latpos  = ((((int(num[2],16)>>2) + (int(num[3],16)<<6) + (int(num[4],16)<<14)) & 0x7FFF)) 
        hzd_longpos = ((((int(num[4],16)>>1) + (int(num[5],16)<<7)) & 0x7FFF))
        hzd_vertpos = ((int(num[6],16) + (int(num[7],16)<<8)) & 0xFFF) 
        return hzd_id, hzd_prob, hzd_latpos, hzd_longpos, hzd_vertpos

    @staticmethod
    def Q4_489BDF(num):
        hzd_h = ((int(num[0],16) + (int(num[1],16)<<8)) & 0x1FF)
        hzd_w = (((int(num[1],16)>>1) + (int(num[2],16)<<7)) & 0x1FF) 
        hzd_latstd = (((int(num[2],16)>>6) + (int(num[3],16)<<2)) & 0xFF) 
        hzd_longstd = (((int(num[3],16)>>6) + (int(num[4],16)<<2)) & 0x1FF) 
        return [hzd_h, hzd_w, hzd_latstd, hzd_longstd]

    @staticmethod
    def Q4_703(num):
        avai = int(num[0],16)>>7
        yaw = ((int(num[2],16) + (int(num[1],16)<<8)) & 0xFFFF)
        yaw = RosbagUtils.convert2signint(yaw, 16) * (-0.00875)
        raw = ((int(num[4],16) + (int(num[3],16)<<8)) & 0xFFFF)
        raw = RosbagUtils.convert2signint(raw, 16)* (-0.00875)
        pitch = ((int(num[6],16) + (int(num[5],16)<<8)) & 0xFFFF)
        pitch = RosbagUtils.convert2signint(pitch, 16)* (-0.00875)
        return avai,pitch,raw,yaw

    @staticmethod
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
        gyro = RosbagUtils.convert2signint(gyro,16) * (-0.00875)
        cam0   = (int(num[7],16)) & 0x01
        cam1   = (int(num[7],16)>>1) & 0x01
        cam2   = (int(num[7],16)>>2) & 0x01
        cam3   = (int(num[7],16)>>3) & 0x01
        cam4   = (int(num[7],16)>>4) & 0x01
        cam5   = (int(num[7],16)>>5) & 0x01
        cam6   = (int(num[7],16)>>6) & 0x01
        cam_set = [cam0,cam1,cam2,cam3,cam4,cam5,cam6]
        return brake, Lblinker, Rblinker, wiper, highbeamon, brake_a, Lblinker_a, Rblinker_a, \
               wiper_a, highbeamon_a, gyro_a, speed_a, speed, gyro ,cam_set

    @staticmethod
    def Q4_720_6(num):
        sign_type       = int(num[0],16) & 0xFF
        suppl_sign_type = int(num[1],16) & 0xFF
        sign_x          = int(num[2],16) & 0xFF
        sign_y          = int(num[3],16) & 0xFF
        sign_z          = int(num[4],16) & 0xFF
        filter_type     = int(num[5],16) & 0x03
        return sign_type, suppl_sign_type, sign_x, sign_y, sign_z, filter_type

    @staticmethod
    def Q4_727(num):
        sign_type1       = int(num[0],16) & 0xFF
        suppl_sign_type1 = int(num[1],16) & 0xFF
        sign_type2       = int(num[2],16) & 0xFF
        suppl_sign_type2 = int(num[3],16) & 0xFF
        sign_type3       = int(num[4],16) & 0xFF
        suppl_sign_type3 = int(num[5],16) & 0xFF
        sign_type4       = int(num[6],16) & 0xFF
        suppl_sign_type4 = int(num[7],16) & 0xFF
        return sign_type1, suppl_sign_type1, sign_type2, suppl_sign_type2,\
               sign_type3, suppl_sign_type3, sign_type4, suppl_sign_type4


class RadarMessageParser:
    @staticmethod
    def Radar_Header_505(num):
        Radar_Frame = (((int(num[5], 16)) << 8) + int(num[6], 16)) & 0xFFFF
        Func_Status = int(num[1], 16) & 0xFF
        AEB_CIPV_ID = int(num[4], 16) & 0xFF
        ACC_CIPV_ID = int(num[3], 16) & 0xFF
        CIPV_ID = int(num[2], 16) & 0xFF
        TunnelFlag = int(num[0], 16) & 0x01
        No_Obj = (int(num[0], 16) >> 2) & 0x3F
        return Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj

    @staticmethod
    def Radar_VehInfo_300(num):
        YawRate_V = (int(num[1], 16) >> 3) & 0x01
        YawRate = ((int(num[2], 16) << 8) + int(num[3], 16) & 0xFFFF) * 0.1 - 100
        VehSpeed_V = (int(num[0], 16) >> 7) & 0x001
        VehSpeed = (((int(num[0], 16) << 4) + (int(num[1], 16) >> 4)) & 0xFFF) * 0.125
        return YawRate_V, YawRate, VehSpeed_V, VehSpeed

    @staticmethod
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
        return Battery_Voltage_too_high, Battery_Voltage_too_low, RF2_Voltage_too_high, RF2_Voltage_too_low, \
               RF1_Voltage_too_high, RF1_Voltage_too_low, MCU_Voltage_too_low, MCU_Voltage_too_high, MCU_Temp_too_low, \
               MCU_Temp_too_high, Lost_Communication_With_Camera, Communication_Bus_Off, Radar_Sleep_Flag

    @staticmethod
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

    @staticmethod
    def Radar_Target_B_509(num):
        Type = (int(num[7], 16) >> 2) & 0x3F
        ProbExist = int(num[5], 16) & 0x03
        DynProp = int(num[3], 16) & 0x07
        MeasStat = (int(num[3], 16) >> 5) & 0x07
        Accel_X = (((int(num[0], 16) << 4) + (int(num[1], 16) >> 4)) & 0xFFF) * 0.04 - 40
        ID = int(num[2], 16) & 0xFF
        MsgCnt_B = int(num[7], 16) & 0x03
        return Type, ProbExist, DynProp, MeasStat, Accel_X, ID, MsgCnt_B


class RosbagParser:
    def __init__(self, ws_path="./", debug=False):
        self.ws_path = ws_path
        self.bag_coordinates = []
        self.sorted_bag_files = sorted(self.get_bag_files(), key=self.sort_key)
        self.data_dict = {}
        self.avaliable_sensor = []
        self.debug = debug

        ### Check Can0 ###
        self.lane_ok = 0
        self.object_ok = False
        self.header_id = None
        self.first_header = None

    def get_bag_files(self):
        return [f for f in os.listdir(self.ws_path) if f.endswith(".bag")]
    
    @staticmethod
    def sort_key(file_name):
        return file_name.split('_')[-1]
    
    @staticmethod
    def initialize_list_data(num_lists):
        return tuple([] for _ in range(num_lists))
    
    def initialize_output_directory(self, root, sub_dir):
        path = os.path.join(root, sub_dir)
        os.makedirs(path, exist_ok=True)
        return path

    def initialze_data_dictionary(self):
        # data_dict 包含了每個 topic 對應的時間戳列表
        data_name = [
            "IVAG5", "MobQ4", "Image", "Imu", "VLS128", "M1Lidar", "Radar"
        ]
        self.data_dict = {data: [] for data in data_name}
    
    def check_avaliable_sensor(self):
        self.avaliable_sensor = []
        for key, value in self.data_dict.items():
            if (not value):
                print(key, end=' ')
            else:
                self.avaliable_sensor.append(key)
        print("not found")

    def process_can0_messages(self, msg, timefloat):
        ### IVA G5 ###
        if (hex(msg.id) == "0x4e1"):
            self.header_id = timefloat
        if (self.header_id and (self.header_id < timefloat)):
            if (hex(msg.id) in {"0x4e2", "0x4e9", "0x4e3", "0x4ea", "0x4e4", "0x4eb", "0x4e5", "0x4ec", "0x4e6"}):
                self.lane_ok += 1
                if (self.lane_ok == 1):
                    self.data_dict["IVAG5"].append(self.header_id)
        else:
            self.lane_ok = 0
        if (hex(msg.id) in {"0x4e7", "0x4e8"}):
            pass

        ### Radar ###
        if (hex(msg.id) == "0x505"):
            self.data_dict["Radar"].append(timefloat)

    def process_can2_messages(self, msg, timefloat):
        if (hex(msg.id) == "0x700"):
            self.data_dict["MobQ4"].append(timefloat)

    def get_nearest_time(self, sensor, time_base):
        for time in self.data_dict[sensor]:
            if (time > time_base):
                return time
        return None

    def get_time_in_range(self, sensor, time_image, pre_time, range):
        if time_image is not None:
            for time in self.data_dict[sensor]:
                time_diff = time_image - time
                if ((pre_time - range/2) < time_diff < (pre_time + range/2)):
                    return time
        return None

    def get_aligned_data(self, time_g5):
        data_list = [time_g5]
        sensor_mapping = {
            "MobQ4": lambda: self.get_nearest_time("MobQ4", time_g5),
            "Image": lambda: self.get_nearest_time("Image", time_g5),
            "Imu": lambda: self.get_nearest_time("Imu", time_g5),
            "VLS128": lambda: self.get_time_in_range("VLS128", time_image, 0.2, 0.1),
            "M1Lidar": lambda: self.get_time_in_range("M1Lidar", time_image, 0.23, 0.1),
            "Radar": lambda: self.get_nearest_time("Radar", time_g5)
        }
        time_image = None
        for sensor, func in sensor_mapping.items():
            if (sensor in self.avaliable_sensor):
                time_value = func()
                if (sensor == "Image"):
                    time_image = time_value
                data_list.append(time_value)

        return tuple(data_list)

    def validate_and_align_data(self):
        time_data_arranged = []
        data_count = 0
        for time_g5 in self.data_dict["IVAG5"]:
            valid = True
            complete = True
            data_tuple = self.get_aligned_data(time_g5)
            # 第一筆資料不用檢查
            if (data_count > 0):
                 # 找看看1個set中有沒有一樣的timestamp
                for i in range(len(data_tuple)):
                    if (time_data_arranged[data_count-1][i] == data_tuple[i]):
                        valid = False
            # 時間對齊上有資料缺失
            if (sum(1 for value in data_tuple if value is not None) != len(self.avaliable_sensor)):
                complete = False

            # 除錯資訊
            if (self.debug):
                print(f"No. {data_count:06d}")
                print(f"Valid\t\t => {valid}")
                print(f"Complete\t => {complete}")
                for i in range(len(data_tuple)):
                    if data_tuple[i] is None:
                        print(f"{self.avaliable_sensor[i]} time\t => None")
                    else:
                        print(f"{self.avaliable_sensor[i]} time\t => {data_tuple[i]:.6f}")
                print("--------------------------------------------------------------------")

            # 只有在資料有效且完整時才將其加入時間數據列表
            if valid and complete:
                time_data_arranged.append(data_tuple)
                data_count += 1

        return time_data_arranged
                
    def process_bag(self, bag):
        ### Add the timestamp of each device to the dictionary ###
        messages = bag.read_messages(topics=[
            "/can0/received_msg", "/can2/received_msg", "/cme_cam", "/imu", "/velodyne_points", "/rslidar_points"
        ])
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (topic == "/can0/received_msg"):
                self.process_can0_messages(msg, timefloat)
            elif (topic == "/can2/received_msg"):
                self.process_can2_messages(msg, timefloat)
            elif (topic == "/cme_cam"):
                self.data_dict["Image"].append(timefloat)
            elif (topic == "/imu"):
                self.data_dict["Imu"].append(timefloat)
            elif (topic == "/velodyne_points"):
                self.data_dict["VLS128"].append(timefloat)
            elif (topic == "/rslidar_points"):
                self.data_dict["M1Lidar"].append(timefloat)

    def process_iva_g5(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        g5_start = False
        g5_path = os.path.join(output_path, "IVA_g5")
        os.makedirs(g5_path, exist_ok=True)

        messages = bag.read_messages(topics="/can0/received_msg")
        g5_l1, g5_l1_info, g5_r1, g5_r1_info, \
        g5_l2, g5_l2_info, g5_r2, g5_r2_info, \
        g5_lane_overview, g5_obj_overview, g5_obj = self.initialize_list_data(11)

        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()

            if (timefloat == time_data_arranged[data_tuples][index]):
                g5_start = True
                timestamp_output = os.path.join(timestamp_path, "IVA_g5.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)
            
            if (g5_start):
                if ((hex(msg.id) == "0x4e1") and ((timefloat - time_data_arranged[data_tuples][index]) > 0.075)):
                    g5_start = False
                    out_file = os.path.join(g5_path, f"{data_tuples:06d}.txt")
                    with open(out_file, 'w') as f:
                        print(','.join(map(str, g5_l1)), file=f)
                        print(','.join(map(str, g5_l1_info)), file=f)
                        print(','.join(map(str, g5_r1)), file=f)
                        print(','.join(map(str, g5_r1_info)), file=f)
                        print(','.join(map(str, g5_l2)), file=f)
                        print(','.join(map(str, g5_l2_info)), file=f)
                        print(','.join(map(str, g5_r2)), file=f)
                        print(','.join(map(str, g5_r2_info)), file=f)
                        print(','.join(map(str, g5_lane_overview)), file=f)
                        print(','.join(map(str, g5_obj_overview)), file=f)
                        print(','.join(map(str, g5_obj)), file=f)
                else:
                    data_str = RosbagUtils.prettyhex2(msg.dlc, msg.data, '-').split('-')

                    ### Lane Host Left ###
                    if (hex(msg.id) == "0x4e2"):
                        quality, exist, c0, c1, c2, c3 = IVAG5MessageParser.IVA_lanecurve_2901379(data_str, flag='1')
                        g5_l1 = quality, exist, c0, c1, c2, c3
                    elif (hex(msg.id) == "0x4e9"):
                        fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVAG5MessageParser.IVA_lanecurve_2902468(data_str)
                        g5_l1_info = fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score
                    ### Lane Host Right ###
                    elif (hex(msg.id) == "0x4e3"):
                        quality, exist, c0, c1, c2, c3 = IVAG5MessageParser.IVA_lanecurve_2901379(data_str, flag='3')
                        g5_r1 = quality, exist, c0, c1, c2, c3
                    elif (hex(msg.id) == "0x4ea"):
                        fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVAG5MessageParser.IVA_lanecurve_2902468(data_str)
                        g5_r1_info = fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score
                    ### Lane Adjacent Left ###
                    elif (hex(msg.id) == "0x4e4"):
                        quality, exist, c0, c1, c2, c3 = IVAG5MessageParser.IVA_lanecurve_2901379(data_str, flag='7')
                        g5_l2 = quality, exist, c0, c1, c2, c3
                    elif (hex(msg.id) == "0x4eb"):
                        fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVAG5MessageParser.IVA_lanecurve_2902468(data_str)
                        g5_l2_info = fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score
                    ### Lane Adjacent Right ###
                    elif (hex(msg.id) == "0x4e5"):
                        quality, exist, c0, c1, c2, c3 = IVAG5MessageParser.IVA_lanecurve_2901379(data_str, flag='9')
                        g5_r2 = quality, exist, c0, c1, c2, c3
                    elif (hex(msg.id) == "0x4ec"):
                        fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score = IVAG5MessageParser.IVA_lanecurve_2902468(data_str)
                        g5_r2_info = fit_score_far, fit_score_mid, fit_score_close, distance_score, detect_score
                    ### Lane Overview ###
                    elif (hex(msg.id) == "0x4e6"):
                        dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, color_L_1st, color_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd, color_L_2nd, color_R_2nd = IVAG5MessageParser.IVA_Lane_Distance_decode(data_str)
                        g5_lane_overview = dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, color_L_1st, color_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd, color_L_2nd, color_R_2nd
                    ### Object Overview ###
                    elif (hex(msg.id) == "0x4e8"):
                        G5obnum, G5lm = IVAG5MessageParser.IVA_obj_290D(data_str)
                        g5_obj_overview = G5obnum, ','.join(map(str, G5lm))
                    ### Object Info ###
                    elif (hex(msg.id) == "0x4e7"):
                        OBJ_id, OBJ_class, OBJ_conf, OBJ_pos_x, OBJ_pos_y, OBJ_vel_x, OBJ_vel_y = IVAG5MessageParser.IVA_obj_290B(data_str)
                        g5_obj.extend([OBJ_id, OBJ_class, OBJ_conf, OBJ_pos_x, OBJ_pos_y, OBJ_vel_x, OBJ_vel_y])

    def process_mobileye_q4(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        q4_start = False
        q4_obj_start = False
        q4_path = os.path.join(output_path, "Mobileye_q4")
        os.makedirs(q4_path, exist_ok=True)

        messages = bag.read_messages(topics="/can2/received_msg")
        q4_l1_info, q4_l1, q4_r1_info, q4_r1, \
        q4_lr_info, q4_lr, q4_rl_info, q4_rl, \
        q4_ll_info, q4_ll, q4_rr_info, q4_rr, \
        q4_obj_overview, q4_obj, q4_obj_pos, q4_obj_other = self.initialize_list_data(16)

        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()

            if (timefloat == time_data_arranged[data_tuples][index]):
                q4_start = True
                q4_obj_start = True
                timestamp_output = os.path.join(timestamp_path, "Mobileye_q4.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)

            if (q4_start):
                if (hex(msg.id) == "0x700" and ((timefloat - time_data_arranged[data_tuples][index]) > 0)):
                    q4_start = False
                    q4_obj_start = False
                    out_file = os.path.join(q4_path, f"{data_tuples:06d}.txt")
                    with open(out_file, 'w') as f:
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

                else:
                    data_str = RosbagUtils.prettyhex2(msg.dlc, msg.data, '-').split('-')
                    ### Lane Host Left ###
                    if (hex(msg.id) == "0x770"):
                        Q4L_conf, Q4L_type, Q4L_side, Q4L_start, Q4L_end = MobQ4MessageParser.Q4_770_772(data_str)
                        q4_l1_info = Q4L_conf, Q4L_type, Q4L_side, Q4L_start, Q4L_end
                    elif (hex(msg.id) == "0x771"):
                        q4_l1 = MobQ4MessageParser.Q4_771_773(data_str)
                    ### Lane Host Right ###
                    elif (hex(msg.id) == "0x772"):
                        Q4R_conf, Q4R_type, Q4R_side, Q4R_start, Q4R_end = MobQ4MessageParser.Q4_770_772(data_str)
                        q4_r1_info = Q4R_conf, Q4R_type, Q4R_side, Q4R_start, Q4R_end
                    elif (hex(msg.id) == "0x773"):
                        q4_r1 = MobQ4MessageParser.Q4_771_773(data_str)
                    ### Lane Adjancent Left Right ###
                    elif (hex(msg.id) == "0x784"):
                        adj_type, adj_conf, adj_start, adj_end, adj_role = MobQ4MessageParser.Q4_78468A(data_str)
                        q4_lr_info = adj_type, adj_conf, adj_start, adj_end, adj_role
                    elif (hex(msg.id) == "0x785"):
                        q4_lr = MobQ4MessageParser.Q4_78579B(data_str)
                        f_adj=1
                    ### Lane Adjancent Right Left ###
                    elif (hex(msg.id) == "0x786"):
                        adj_type1, adj_conf1, adj_start1, adj_end1, adj_role1 = MobQ4MessageParser.Q4_78468A(data_str)
                        q4_rl_info = adj_type1, adj_conf1, adj_start1, adj_end1, adj_role1
                    elif (hex(msg.id) == "0x787"):
                        q4_rl = MobQ4MessageParser.Q4_78579B(data_str)
                        f_adj1=1
                    ### Lane Adjancent Left Left ###
                    elif (hex(msg.id) == "0x788"):
                        adj_typ2, adj_conf2, adj_start2, adj_end2, adj_role2 = MobQ4MessageParser.Q4_78468A(data_str)
                        q4_ll_info = adj_typ2, adj_conf2, adj_start2, adj_end2, adj_role2
                    elif (hex(msg.id) == "0x789"):
                        q4_ll = MobQ4MessageParser.Q4_78579B(data_str)
                        f_adj2=1
                    ### Lane Adjancent Right Right ###
                    elif (hex(msg.id) == "0x78a"):
                        adj_typ3, adj_conf3, adj_start3, adj_end3, adj_role3 = MobQ4MessageParser.Q4_78468A(data_str)
                        q4_rr_info = adj_typ3, adj_conf3, adj_start3, adj_end3, adj_role3
                    elif (hex(msg.id) == "0x78b"):
                        q4_rr = MobQ4MessageParser.Q4_78579B(data_str)
                        f_adj3=1
                    ### Object Overview ###
                    elif (hex(msg.id) == "0x541"):
                        # "CIPV" 通常指的是 "Critical Instant of Possible Collision Vehicle"，也就是可能發生碰撞的關鍵瞬間車輛。
                        Mob_Num_Objs, CIPV, CIPVlost, VDacc = MobQ4MessageParser.Q4_541(data_str)
                        q4_obj_overview = Mob_Num_Objs, CIPV, CIPVlost, VDacc
                    ### Object Info ###
                    elif (hex(msg.id) in ["0x500", "0x503", "0x506", "0x509", "0x50c", "0x50f", "0x512", "0x515", "0x518", "0x51b", "0x51e", "0x521"]):
                        iddd, Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, OBJ_Lane_Ass, Relative_Lat_Velocity = MobQ4MessageParser.Q4_500(data_str)
                        if (iddd != 0):
                            q4_obj_start = True
                            q4_obj.extend((iddd, Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, OBJ_Lane_Ass, Relative_Lat_Velocity))
                        else:
                            q4_obj_start = False
                    elif (hex(msg.id) in ["0x501", "0x504", "0x507", "0x50a", "0x50d", "0x50g", "0x510", "0x513", "0x516", "0x519", "0x51c", "0x51f", "0x522"]):
                        Absolute_Long_Acc, Long_Distance, Lateral_Distance = MobQ4MessageParser.Q4_501(data_str)
                        if (q4_obj_start):
                            q4_obj_pos.extend((Absolute_Long_Acc, Long_Distance, Lateral_Distance))
                    elif (hex(msg.id) in ["0x502", "0x505", "0x508", "0x50b", "0x50e", "0x511", "0x514", "0x517", "0x51a", "0x51d", "0x520", "0x523"]):
                        Absolute_Speed, OBJ_Motion_Status, OBJ_Motion_Category, Brake_Light, Turn_Indicator_Right, Turn_Indicator_Left, Light_indicator_validity, OBJ_Angle_Mid, OBJ_Angle_Rate = MobQ4MessageParser.Q4_502(data_str)
                        if (q4_obj_start):
                            q4_obj_other.extend((Absolute_Speed, OBJ_Angle_Mid, OBJ_Angle_Rate))
                    elif (hex(msg.id) in ["0x720", "0x721", "0x722", "0x723", "0x724", "0x725", "0x726"]):
                        Mob_tsr0 = MobQ4MessageParser.Q4_720_6(data_str)
                    elif (hex(msg.id) == "0x727"):
                        Mob_Sign_type = MobQ4MessageParser.Q4_727(data_str)

    def process_image(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        image_path = os.path.join(output_path, "image")
        os.makedirs(image_path, exist_ok=True)

        messages = bag.read_messages(topics="/cme_cam")
        
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (timefloat == time_data_arranged[data_tuples][index]):

                timestamp_output = os.path.join(timestamp_path, "image.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)

                img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
                image_output = os.path.join(image_path, f"{data_tuples:06d}.png")
                cv2.imwrite(image_output, img)

    def process_imu(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        imu_start = False
        imu_path = os.path.join(output_path, "imu")
        os.makedirs(imu_path, exist_ok=True)

        messages = bag.read_messages(topics="/imu")
        
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (timefloat == time_data_arranged[data_tuples][index]):
                imu_start = True
            
            if (imu_start):
                if (timefloat == time_data_arranged[data_tuples + 1][index]):
                    imu_start = False
                else:
                    timestamp_output = os.path.join(timestamp_path, "imu.txt")
                    with open(timestamp_output, 'a+') as time_f:
                        print(timefloat, file=time_f)

                    out_file = os.path.join(imu_path, f"{data_tuples:06d}.txt")
                    with open(out_file, 'a+') as f:
                        print(
                            f"{msg.orientation.x},{msg.orientation.y},{msg.orientation.z},"
                            f"{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},"
                            f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}", \
                            file=f
                        )

    def process_m1lidar(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        m1lidar_path = os.path.join(output_path, "VLS128_pcdnpy")
        os.makedirs(m1lidar_path, exist_ok=True)

        messages = bag.read_messages(topics="/rslidar_points")
        
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (timefloat == time_data_arranged[data_tuples][index]):
                timestamp_output = os.path.join(timestamp_path, "m1lidar.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)
                
                gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                points = np.array(list(gen))
                points_xyz = points[:, :3]
                points_other = points[:, 3:]
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_xyz)

                out_file = os.path.join(m1lidar_path, f"{data_tuples:06d}.npy")
                np.save(out_file, points)
                out_file = os.path.join(m1lidar_path, f"{data_tuples:06d}.pcd")
                o3d.io.write_point_cloud(out_file, pcd)

    def process_vls128(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        vls128_path = os.path.join(output_path, "VLS128_pcdnpy")
        os.makedirs(vls128_path, exist_ok=True)

        messages = bag.read_messages(topics="/velodyne_points")
        
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (timefloat == time_data_arranged[data_tuples][index]):

                timestamp_output = os.path.join(timestamp_path, "VLS128.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)
                
                gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)
                points = np.array(list(gen))
                points_xyz = points[:, :3]
                points_other = points[:, 3:]
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_xyz)

                out_file = os.path.join(vls128_path, f"{data_tuples:06d}.npy")
                np.save(out_file, points_other)
                out_file = os.path.join(vls128_path, f"{data_tuples:06d}.pcd")
                o3d.io.write_point_cloud(out_file, pcd)

    def process_radar(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path, index):
        radar_start = False
        radar_path = os.path.join(output_path, "radar")
        os.makedirs(radar_path, exist_ok=True)

        messages = bag.read_messages(topics="/can0/received_msg")
        radar_header, radar_targetA, radar_targetB = self.initialize_list_data(3)
        
        for topic, msg, t in messages:
            timefloat = msg.header.stamp.to_sec()
            if (timefloat == time_data_arranged[data_tuples][index]):
                radar_start = True

                timestamp_output = os.path.join(timestamp_path, "radar.txt")
                with open(timestamp_output, 'a+') as time_f:
                    print(timefloat, file=time_f)

                out_file = os.path.join(radar_path, f"{data_tuples:06d}.txt")
                f = open(out_file, 'w')

            if (radar_start):
                if (hex(msg.id) == "0x505" and ((timefloat - time_data_arranged[data_tuples][index]) > 0)):
                    radar_start = False
                    f.close()

                else:
                    data_str = RosbagUtils.prettyhex2(msg.dlc, msg.data, '-').split('-')
                    ### Radar Header ###
                    if (hex(msg.id) == "0x505"):
                        Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj = RadarMessageParser.Radar_Header_505(data_str)
                        radar_header = Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj
                        print(','.join(map(str, radar_header)), file=f)
                    ### Radar Object A ###
                    elif (hex(msg.id) in {"0x508", "0x50a", "0x50c", "0x50e", "0x510", "0x512", "0x514", "0x516", "0x518", "0x51a", "0x51c", "0x51e", "0x520", "0x522", "0x524", "0x526", "0x528", "0x52a", "0x52c", "0x52e", "0x530", "0x532", "0x534", "0x536", "0x538", "0x53a", "0x53c", "0x53e", "0x540", "0x542", "0x544", "0x546"}):
                        AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID_A, MsgCnt_A = RadarMessageParser.Radar_Target_A_508(data_str)
                        radar_targetA = AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID_A, MsgCnt_A
                    ### Radar Object B ###
                    elif (hex(msg.id) in {"0x509", "0x50b", "0x50d", "0x50f", "0x511", "0x513", "0x515", "0x517", "0x519", "0x51b", "0x51d", "0x51f", "0x521", "0x523", "0x525", "0x527", "0x529", "0x52b", "0x52d", "0x52f", "0x531", "0x533", "0x535", "0x537", "0x539", "0x53b", "0x53d", "0x53f", "0x541", "0x543", "0x545", "0x547"}):
                        Type, ProbExist, DynProp, MeasStat, Accel_X, ID_B, MsgCnt_B = RadarMessageParser.Radar_Target_B_509(data_str)
                        radar_targetB = Type, ProbExist, DynProp, MeasStat, Accel_X, ID_B, MsgCnt_B
                        if (not radar_targetA):
                            print(f"No. {data_tuples:06d}: missing {hex(msg.id-1)}")
                        if (radar_targetA and radar_targetB): # 兩個都不是空的
                            if (radar_targetA[7] == radar_targetB[5]): # ID相同
                                print(','.join(map(str, radar_targetA[0:8] + radar_targetB[0:5])), file=f)
                            else:
                                print(f"No. {data_tuples:06d}: different targetID {hex(msg.id-1)} {hex(msg.id)}")

    def process_bag_for_timestamps(self, bag, data_tuples, time_data_arranged, timestamp_path, output_path):
        ### Process messages from each sensor ###
        for sensor in self.avaliable_sensor:
            index = self.avaliable_sensor.index(sensor)
            if (sensor == "IVAG5"):
                self.process_iva_g5(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "MobQ4"):
                self.process_mobileye_q4(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "Image"):
                self.process_image(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "Imu"):
                self.process_imu(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "M1Lidar"):
                self.process_m1lidar(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "VLS128"):
                self.process_vls128(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
            elif (sensor == "Radar"):
                self.process_radar(bag, data_tuples, time_data_arranged, timestamp_path, output_path, index)
    
    def process_gps(self, bag_file, output_path):
        with rosbag.Bag(bag_file, 'r') as bag:
            out_file = os.path.join(output_path, "gps.txt")

            for toipc, msg, t in bag.read_messages(topics=["/fix"]):

                with open(out_file, '+a') as f:
                    print(f"{msg.latitude},{msg.longitude},{msg.altitude},{','.join(map(str, msg.position_covariance))}", file=f)
                    if (f"{msg.latitude}" != "nan"):
                        self.bag_coordinates.append({
                            "coordinates": [msg.latitude, msg.longitude],
                            "bag_name": os.path.basename(bag_file).split('.')[0]
                        })

    def generate_map(self):
        if (self.bag_coordinates):
            taiwan_map = folium.Map(location=self.bag_coordinates[0]["coordinates"], zoom_start=16, tiles="OpenStreetMap")
            for c in self.bag_coordinates:
                folium.Marker(
                    location=c["coordinates"],
                    popup=c["bag_name"],
                    icon=folium.Icon(color="black", icon_color="red", icon="car", prefix="fa")
                ).add_to(taiwan_map)

                folium.PolyLine(
                    locations=[c["coordinates"] for c in self.bag_coordinates],
                    color="blue",
                    weight=3
                ).add_to(taiwan_map)

            # 顯示地圖 以父目錄名稱命名
            parent_dir_name = os.path.basename(os.path.dirname(os.path.realpath(__file__)))
            taiwan_map.save(f"{parent_dir_name}.html")
            print("Map DONE !")

    def parse_bags(self):
        for files in tqdm(self.sorted_bag_files, desc="Parsing all bags", ascii=True):
            bag_file = os.path.join(self.ws_path, files)
            self.initialze_data_dictionary()

            ### Open Bag ###
            try:
                with rosbag.Bag(bag_file, 'r') as bag:
                    self.process_bag(bag)
            except rosbag.bag.ROSBagException as e:
                print(f"Error opening ROS Bag file: {e}")
                continue
            
            self.check_avaliable_sensor()
            time_data_arranged = self.validate_and_align_data()

            ### 從 bag 讀取 time_data_arranged timestamp 的資料 順便把最後一筆資料濾掉 ###
            for data_tuples in tqdm(range (len(time_data_arranged) - 1), desc=f"Parsing {files}", ascii=True):
                ### Open Bag ###
                try:
                    with rosbag.Bag(bag_file, 'r') as bag:
                        bag_name = files.split('.')[0]
                        bag_date = datetime.datetime.fromtimestamp(bag.get_start_time()).date()
                        ### Output dir ###
                        parsed_path = self.initialize_output_directory(self.ws_path, f"{bag_date}_parsed_data")
                        bag_path = self.initialize_output_directory(self.ws_path, f"{bag_date}_bag")
                        output_path = self.initialize_output_directory(parsed_path, bag_name)
                        timestamp_path = self.initialize_output_directory(output_path, "timestamp")
                        ### Process ###
                        self.process_bag_for_timestamps(bag, data_tuples, time_data_arranged, timestamp_path, output_path)
                except rosbag.bag.ROSBagException as e:
                    print(f"Error opening ROS Bag file: {e}")
                    continue
            self.process_gps(bag_file, output_path)
            shutil.move(bag_file, bag_path)
            # print(f"Finished\t{files} !!!!!!!")
        self.generate_map()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse ROS bag files.")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to print detailed information.")
    args = parser.parse_args()

    parser = RosbagParser(ws_path="./", debug=args.debug)
    parser.parse_bags()
