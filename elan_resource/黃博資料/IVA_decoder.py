import numpy as np
import sys

def killzero(a):
    c = np.where(a==0)
    b = np.delete(a, c)
    
    return b

def IVA_lane_1st_decode(can_data):
    Factor = np.array([0.00390625,0.000010895,0.00000061037,0.00000000366217])
    offset = np.array([-8,-0.357,-0.02,-0.00012])
    
    Exist = (can_data[0] & 0x08)>>3
    reality = can_data[0] & 0x07
    
    C0 = (can_data[0]>>4)+(can_data[1]<<4)
    C0 = ((C0*Factor[0])+offset[0])
    
    C1 = can_data[2]+(can_data[3]<<8)
    C1 = (C1*Factor[1])+offset[1]
    
    C2 = can_data[4]+(can_data[5]<<8)
    C2 = (C2*Factor[2])+offset[2]
    
    C3 = can_data[6]+(can_data[7]<<8)
    C3 = (C3*Factor[3])+offset[3]
    
    return [C0,C1,C2,C3,Exist,reality]

def IVA_lane_2nd_decode(can_data,side):
    Factor = np.array([0.00390625,0.000010895,0.00000061037,0.00000000366217])
    offset = np.array([-8,-0.357,-0.02,-0.00012])
    
    Exist = (can_data[0] & 0x08)>>3
    reality = can_data[0] & 0x07
    
    if (side=='L'):
        C0 = (can_data[0]>>4)+(can_data[1]<<4)
        C0 = ((C0*Factor[0])+(-18))
    elif (side=='R'):
        C0 = (can_data[0]>>4)+(can_data[1]<<4)
        C0 = ((C0*Factor[0])+ 2)
    else:
        print("[IVA] lane_2nd decoder side Error !!!")
        sys.exit()
        
    C1 = can_data[2]+(can_data[3]<<8)
    C1 = (C1*Factor[1])+offset[1]
    
    C2 = can_data[4]+(can_data[5]<<8)
    C2 = (C2*Factor[2])+offset[2]
    
    C3 = can_data[6]+(can_data[7]<<8)
    C3 = (C3*Factor[3])+offset[3]
    
    return [C0,C1,C2,C3,Exist,reality]

def IVA_RoadSide_decode(can_data,side):
    Factor = np.array([0.00390625,0.000010895,0.00000061037,0.00000000366217])
    offset = np.array([-8,-0.357,-0.02,-0.00012])
    
    Exist = (can_data[0] & 0x08)>>3
    reality = can_data[0] & 0x07
    
    if (side=='L'):
        C0 = (can_data[0]>>4)+(can_data[1]<<4)
        C0 = ((C0*Factor[0])+(-16))
    elif (side=='R'):
        C0 = (can_data[0]>>4)+(can_data[1]<<4)
        C0 = ((C0*Factor[0]))
    else:
        print("[IVA] RoadSide decoder side Error !!!")
        sys.exit()
        
    C1 = can_data[2]+(can_data[3]<<8)
    C1 = (C1*Factor[1])+offset[1]
    
    C2 = can_data[4]+(can_data[5]<<8)
    C2 = (C2*Factor[2])+offset[2]
    
    return [C0,C1,C2,Exist,reality]

def IVA_Lane_Distance_decode(can_data):
    dis_L_1st = can_data[0]
    dis_R_1st = can_data[1]
    type_L_1st = can_data[2]&0x0F
    type_R_1st = (can_data[2]&0xF0)>>4
    
    dis_L_2nd = can_data[4]
    dis_R_2nd = can_data[5]
    type_L_2nd = can_data[6]&0x0F
    type_R_2nd = (can_data[6]&0xF0)>>4
    
    return [dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd]

def IVA_obj_decode(can_data):
    ID = can_data[0] & 0x3F
    classes = ((can_data[0]>>6) + (can_data[1]<<2)) & 0x0F
    
    Y = ((can_data[2]>>2)+(can_data[3]<<6)) & 0xFFF
    Y = Y * 0.03125
    
    X = ((can_data[3]>>6)+(can_data[4]<<2) + (can_data[5]<<10)) & 0xFFF
    X = (X * 0.015625)-32
    
    Y_vel = ((can_data[5]>>2) + (can_data[6]<<6)) & 0x7FF  #前車縱向車速
    Y_vel = (Y_vel * 0.01953125)-20
    
    X_vel = ((can_data[6]>>5) + (can_data[7]<<3)) & 0x7FF  #前車橫像車速
    X_vel = (X_vel * 0.01953125)-20
    
    return [ID, classes, X, Y, Y_vel, X_vel]

def IVA_obj_num_decode(can_data):
    number = can_data[0]
    
    return [number]

def IVA_warning_type(can_data):  #id=0x4E1 發報訊號
    
    IVA_VB_warning = (can_data[0] & 0x08)>>3
    IVA_PD_warning = (can_data[0] & 0x10)>>4
    IVA_L_LD_warning = (can_data[0] & 0x20)>>5   #  (can_data[0]>>5) & 0x01
    IVA_R_LD_warning = (can_data[0] & 0x40)>>6
    IVA_FCW_warning = (can_data[0] & 0x80)>>7
    
    TTC_time = can_data[1] & 0x7F
    TTC_time = (TTC_time *0.1) -0.1
    
    IVA_Relevant_object = ((can_data[1]>>7) + (can_data[2]<<1)) & 0x07
    IVA_warning_level = (can_data[2]>>2) & 0x0F
    
    IVA_TTLC_L_0m = ((can_data[2]>>6) + (can_data[3]<<2)) & 0x7F   #Time to Lane collision
    IVA_TTLC_R_0m = ((can_data[3]>>5) + (can_data[4]<<3)) & 0x7F
    IVA_TTLC_L_10m = ((can_data[4]>>4) + (can_data[5]<<4)) & 0x7F
    IVA_TTLC_R_10m = ((can_data[5]>>3) + (can_data[6]<<5)) & 0x7F
    IVA_TTLC_L_20m = ((can_data[6]>>2) + (can_data[7]<<6)) & 0x7F
    IVA_TTLC_R_20m = (can_data[7]>>1) & 0x7F
    
    return [IVA_VB_warning, IVA_PD_warning, IVA_L_LD_warning, IVA_R_LD_warning, IVA_FCW_warning, TTC_time]
    
    
    
    


