import numpy as np
import os
import rosbag

def prettyhex2(dlc,nums, sep=''):
    return sep.join(f'{a:02x}' for a in nums[0:dlc])

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

bagpath = "./"

bag_files = [f for f in os.listdir(bagpath) if f.endswith(".bag")]
sorted_bag_files = sorted(bag_files)

for files in sorted_bag_files:
    if files[-4:] == ".bag":
        bag_file = os.path.join(bagpath, files)
        bag_name = files.split('.')[0]

        output_path = os.path.join(bagpath, bag_name)
        radar_path = os.path.join(output_path, "radar")

        if not os.path.exists(radar_path):
            os.makedirs(radar_path)

        try:
            bag = rosbag.Bag(bag_file, "r")
        except rosbag.bag.ROSBagException as e:
            print(f"Error opening ROS Bag file: {e}")
            continue

        testdata_output = os.path.join(radar_path, "radar.txt")

        for topic, msg, t in bag.read_messages(topics="/can0/received_msg"):

            timeq = msg.header.stamp.to_sec()
            timestr = "{:.6f}".format(timeq)
            id = hex(msg.id)
            data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

            f = open(testdata_output, 'a+')

            if (id == "0x300"):
                YawRate_V, YawRate, VehSpeed_V, VehSpeed = Radar_VehInfo_300(data_str)
                data = f"timestamp = {timestr}\tYawRate_V = {YawRate_V}\tYawRate = {YawRate}\tVehSpeed_V = {VehSpeed_V}\tVehSpeed = {VehSpeed}"
                
                print(data, file=f)

            elif (id == "0x505"):
                Radar_Frame, Func_Status, AEB_CIPV_ID, ACC_CIPV_ID, TunnelFlag, No_Obj = Radar_Header_505(data_str)
                data = f"timestamp = {timestr}\tRadar_Frame = {Radar_Frame}\tFunc_Status = {Func_Status}\tAEB_CIPV_ID = {AEB_CIPV_ID}\tACC_CIPV_ID = {ACC_CIPV_ID}\tTunnelFlag = {TunnelFlag}\tNo_Obj = {No_Obj}"

                print(data, file=f)

            elif (id == "0x506"):
                Battery_Voltage_too_high, Battery_Voltage_too_low, RF2_Voltage_too_high, RF2_Voltage_too_low, RF1_Voltage_too_high, RF1_Voltage_too_low, MCU_Voltage_too_low, MCU_Voltage_too_high, MCU_Temp_too_low, MCU_Temp_too_high, Lost_Communication_With_Camera, Communication_Bus_Off, Radar_Sleep_Flag = Radar_Sta_506(data_str)
                data = f"timestamp = {timestr}\tBattery_Voltage_too_high = {Battery_Voltage_too_high}\tBattery_Voltage_too_low = {Battery_Voltage_too_low}\tRF2_Voltage_too_high = {RF2_Voltage_too_high}\tRF2_Voltage_too_low = {RF2_Voltage_too_low}\tRF1_Voltage_too_high = {RF1_Voltage_too_high}\tRF1_Voltage_too_low = {RF1_Voltage_too_low}\tMCU_Voltage_too_low = {MCU_Voltage_too_low}\tMCU_Voltage_too_high = {MCU_Voltage_too_high}\tMCU_Temp_too_low = {MCU_Temp_too_low}\tMCU_Temp_too_high = {MCU_Temp_too_high}\tLost_Communication_With_Camera = {Lost_Communication_With_Camera}\tCommunication_Bus_Off = {Communication_Bus_Off}\tRadar_Sleep_Flag = {Radar_Sleep_Flag}"

                print(data, file=f)

            elif (id in {"0x508", "0x50a", "0x50c", "0x50e", "0x510", "0x512", "0x514", "0x516", "0x518", "0x51a", "0x51c", "0x51e", "0x520", "0x522", "0x524", "0x526", "0x528", "0x52a", "0x52c", "0x52e", "0x530", "0x532", "0x534", "0x536", "0x538", "0x53a", "0x53c", "0x53e", "0x540", "0x542", "0x544", "0x546"}):
                AEB_CIPVFlag, ACC_CIPVFlag, CIPVFlag, Vel_Y, Vel_X, Pos_Y, Pos_X, ID, MsgCnt_A = Radar_Target_A_508(data_str)
                data = f"timestamp = {timestr}\tAEB_CIPVFlag = {AEB_CIPVFlag}\tACC_CIPVFlag = {ACC_CIPVFlag}\tCIPVFlag = {CIPVFlag}\tVel_Y = {Vel_Y:.06f}\tVel_X = {Vel_X:.06f}\tPos_Y = {Pos_Y:.03f}\tPos_X = {Pos_X:.03f}\tID = {ID}\tMsgCnt_A = {MsgCnt_A}"

                print(data, file=f)

            elif (id in {"0x509", "0x50b", "0x50d", "0x50f", "0x511", "0x513", "0x515", "0x517", "0x519", "0x51b", "0x51d", "0x51f", "0x521", "0x523", "0x525", "0x527", "0x529", "0x52b", "0x52d", "0x52f", "0x531", "0x533", "0x535", "0x537", "0x539", "0x53b", "0x53d", "0x53f", "0x541", "0x543", "0x545", "0x547"}):
                Type, ProbExist, DynProp, MeasStat, Accel_X, ID, MsgCnt_B = Radar_Target_B_509(data_str)
                data = f"timestamp = {timestr}\tType = {Type}\tProbExist = {ProbExist}\tDynProp = {DynProp}\tMeasStat = {MeasStat}\tAccel_X = {Accel_X:.06f}\tID = {ID}\tMsgCnt_B = {MsgCnt_B}"
            
                print(data, file=f)
            
            f.close()

        bag.close()
        print(f"Finished\t{files} !!!!!!!")