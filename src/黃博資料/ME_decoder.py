import numpy as np
import sys

def Q4_lane_decoder(num):
    
    Factor = np.array([0.01, 0.000977, 0.000000977, 0.00000000373])
    offset = np.array([-10,-0.357,-0.032,-0.000122])
    
    C0 = (num[0]+(num[1]<<8))&0x7FF  #8+3
    C0 = ((C0*Factor[0])+offset[0])
    
    C1 = (num[1]>>3)+(num[2]<<5)&0x3FF  #5+5
    C1 = (C1*Factor[1])+offset[1]
    
    C2 = ((num[2]>>5)+(num[3]<<3)+(num[4]<<11))&0xFFFF #3+8+5
    C2 = (C2*Factor[2])+offset[2]
    
    C3 = ((num[4]>>5)+(num[5]<<3)+(num[6]<<11))&0xFFFF #3+8+5
    C3 = (C3*Factor[3])+offset[3]
    
    return [C0,C1,C2,C3]

def Q4_lane_info_decoder(can_data):
    conf = int(can_data[0]%128)
    LH_Start = (int(can_data[3]%16)<<11) + (can_data[2]<<3) + int(can_data[1]/32)
    LH_End = (int(can_data[5]%8)<<12) + (can_data[4]<<4) + int(can_data[3]/16)
    distance = int((LH_End*0.01) - (LH_Start*0.01))
    Type = (int(can_data[1]%8)<<1) + int(can_data[0]/128)
    
    return [conf, distance, Type]

def Q4_Obj_infoA_decoder(can_data):
    
    Obj_class = int(int(can_data[1]%4)<<1) + int(can_data[0]/128)
    
    OBJ_W = int(int(can_data[2]%2)<<6) + int(can_data[1]/4)
    OBJ_W = OBJ_W*0.05
    
    OBJ_L = int(int(can_data[3]%4)<<7) + int(can_data[2]/2)
    OBJ_L = OBJ_L*0.05
    
    Relative_Long_Velocity = int(int(can_data[4]%128)<<6) + int(can_data[3]/4)
    Relative_Long_Velocity = (Relative_Long_Velocity*0.05) - 120
    
    OBJ_Lane_Ass = int(int(can_data[5]%4)<<1) + int(can_data[4]/128)
    
    Relative_Lat_Velocity = int(int(can_data[6]%32)<<6) + int(can_data[5]/4)
    Relative_Lat_Velocity = (Relative_Lat_Velocity*0.05) - 50
    
    
    return [Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, Relative_Lat_Velocity, OBJ_Lane_Ass]

def Q4_Obj_infoB_decoder(can_data):
    Absolute_Long_Acc = int(int(can_data[1]%2)<<8) + can_data[0]
    Absolute_Long_Acc = (Absolute_Long_Acc*0.05) - 12.8 
    
    Long_Distance_Acc = int(int(can_data[2]%64)<<7) + int(can_data[1]/2)
    Long_Distance_Acc = (Long_Distance_Acc*0.05)
    
    Lateral_Distance = int(int(can_data[4]%4)<<10)  + (int(can_data[3])<<2) + int(can_data[2]/64)
    Lateral_Distance = (Lateral_Distance*0.05) - 102.4
    
    return [Lateral_Distance,Long_Distance_Acc,Absolute_Long_Acc]

def Q4_Obj_infoC_decoder(can_data):
    Absolute_Speed = int(int(can_data[1]%16)<<8) + can_data[0]
    Absolute_Speed = (Absolute_Speed*0.05) - 100
    
    OBJ_Motion_Status = int(int(can_data[1]>>4)%8)
    OBJ_Motion_Category = int(int(can_data[2]%8)<<1) + int(can_data[1]/128)
    Brake_Light = int(int(can_data[2]>>3)%2)
    Turn_Indicator_Right = int(int(can_data[2]>>4)%2)
    Turn_Indicator_Left = int(int(can_data[2]>>5)%2)
    Light_indicator_validity = int(int(can_data[2]>>6)%2)
    
    OBJ_Angle_Mid = int(int(can_data[4]%32)<<9) + int(can_data[3]<<1) + int(can_data[2]/128)
    OBJ_Angle_Mid = (OBJ_Angle_Mid*0.0002) - 1.571
    
    OBJ_Angle_Rate = int(int(can_data[6]%2)<<12) + int(can_data[5]<<3) + int(can_data[4]/32)
    OBJ_Angle_Rate = (OBJ_Angle_Rate*0.002) - 2.234 
           
    return [Absolute_Speed,OBJ_Motion_Status,OBJ_Motion_Category,Brake_Light,Turn_Indicator_Right,Turn_Indicator_Left,Light_indicator_validity,OBJ_Angle_Mid,OBJ_Angle_Rate]


    
def toSigned16(n):
    n = n & 0xffff
    return (n ^ 0x8000) - 0x8000

def Q2_lane_decoder(can_data):
    
    Type = can_data[0]&0x0F
    conf = (can_data[0]&0x30)>>4
    
    C0 = can_data[1]+(can_data[2]<<8)
    C0 = toSigned16(C0)/256
    
    C2 = can_data[3]+(can_data[4]<<8)
    C2 = (C2-0x7FFF)/1024/1000
    
    C3 = can_data[5]+(can_data[6]<<8)
    C3 = (C3-0x7FFF)/(1<<28)    
    
    return [C0,C2,C3,Type,conf]


def Q2_lane_info_decoder(can_data):
    C1 = can_data[0]+(can_data[1]<<8)
    C1 = (C1-0x7FFF)/1024
    
    distance =  (can_data[2]+(can_data[3]<<8))&0x7FFF
    distance = distance/256
    
    return [C1,distance]

def Q4_Gyro_rate_data(can_data): #0x703 
    Gyro_availability = (can_data[0]>>7) & 0x01
    
    yaw_rate = (can_data[1]<<8) + (can_data[2])
    yaw_rate = yaw_rate*(-0.00875)
    
    roll_rate = (can_data[3]<<8) + (can_data[4])
    roll_rate = roll_rate*(-0.00875)
    
    pitch_rate = (can_data[5]<<8) + (can_data[6])
    pitch_rate = pitch_rate*(-0.00875)
    
    return [yaw_rate, roll_rate, pitch_rate]

def Q4_warning(can_data):   #0x700 發報訊號
    sound_type = can_data[0] & 0x07  #1:L-LD,2:R-LD,3:HMW,4:SLI,5:UFCW,6:FCW&PCW
    
    L_LD_availability = can_data[4] & 0x01
    L_LDW = (can_data[4]>>1) & 0x01
    
    R_LD_availability = can_data[5] & 0x01
    R_LDW = (can_data[4]>>2) & 0x01
    
    FCW = (can_data[4]>>3) & 0x01
    PCW = (can_data[5]>>1) & 0x01
    ped_in_dangerzone = (can_data[5]>>2) & 0x01
    
    tamper_alert = (can_data[5]>>5) & 0x01
    HW_warning_level = can_data[7] & 0x3
    
    return [FCW, PCW, L_LDW, R_LDW, ped_in_dangerzone,HW_warning_level, sound_type]
    
def Q4_obj_num(can_data): #0x541
    number=can_data[1] & 0x0F
    number = int(number%16)
    return [number]