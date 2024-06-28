import math
import numpy as np
import os
import rosbag
import cv2
import shutil

from sensor_msgs import point_cloud2

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
    
    dis_L_2nd = int(can_data[4], 16)
    dis_R_2nd = int(can_data[5], 16)
    type_L_2nd = int(can_data[6], 16) & 0x0F
    type_R_2nd = (int(can_data[6], 16) & 0xF0) >> 4
    
    return [dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd]

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
    
    return quality, exist, [c0, c1, c2, c3]

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

    return [objnum, CIPV, CIPVlost,acc]

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

class Node:
    def __init__(self, name, ss=""):
        self._start = "<" + name + ">"
        self._content = []
        if ss:
            self._content.append(ss)
        self._end = "</" + name + ">"
    
    def add(self, node):
        self._content.append(node)
    
    def printstr(self, indent=""):
        if self._content and not isinstance(self._content[0], Node):
            print(indent + self._start + str(self._content[0]) + self._end)
        else:
            print(indent + self._start)
            for item in self._content:
                item.printstr(indent + "\t")
            print(indent + self._end)

    def writexml(self, f, indent=""):
        if self._content and not isinstance(self._content[0], Node):
            print(indent + self._start + str(self._content[0]) + self._end, file=f)
        else:
            print(indent + self._start, file=f)
            for item in self._content:
                item.writexml(f, indent + "\t")
            print(indent + self._end, file=f)

dirs = "/home/evil/Desktop/Car/bag_parse"
bagpath = './'

for files in os.listdir(bagpath):
    if files[-4:] == ".bag":
        print(f"parsing\t{files} ......")
        bag_file = os.path.join(bagpath, files)
        bag_name = files.split('.')[0]

        # Output dir
        output_path = os.path.join(bagpath, bag_name)
        res_path = os.path.join(output_path, "save")
        xml_path = os.path.join(output_path, "Annotations")
        img_path = os.path.join(output_path, "ImageSets")

        if not os.path.exists(output_path):
            os.makedirs(output_path)
        if not os.path.exists(res_path):
            os.makedirs(res_path)
        if not os.path.exists(xml_path):
            os.makedirs(xml_path)
        if not os.path.exists(img_path):
            os.makedirs(img_path)        

        # OPEN BAG
        try:
            bag = rosbag.Bag(bag_file, "r")
        except rosbag.bag.ROSBagException as e:
            print(f"Error opening ROS Bag file: {e}")
            continue

        topic_to_read = ["/can0/received_msg", "/can2/received_msg", "/cme_cam", "/imu", "/rslidar_points"]
        topic_data_dict = {topic: [] for topic in topic_to_read}

        # topic_data_dict 包含了每個 topic 對應的時間戳列表
        messages = bag.read_messages(topics=topic_to_read)
        for topic, msg, t in messages:
            
            timefloat = msg.header.stamp.to_sec()
            timestr = "{:.6f}".format(timefloat)

            if (topic == "/can0/received_msg"):
                if (hex(msg.id) == "0x4e1"):
                    temp_0x4e1 = timefloat
                if (hex(msg.id) == "0x4e2"):
                    topic_data_dict[topic].append(temp_0x4e1)

            elif (topic == "/can2/received_msg"):
                if (hex(msg.id) == "0x700"):
                    topic_data_dict[topic].append(timefloat)

            elif (topic == "/cme_cam"):
                topic_data_dict[topic].append(timefloat)
                
            elif (topic == "/imu"):
                topic_data_dict[topic].append(timefloat)
                
            elif (topic == "/rslidar_points"):
                topic_data_dict[topic].append(timefloat)
        
        bag.close()


        # 從 topic_data_dict 挑出想要的 timestamp 存到 time_topic_arranged
        time_g5_current = 0
        time_q4_current = 0
        time_image_current = 0
        time_imu_current = 0
        time_lidar_current = 0
        time_topic_arranged = []

        for time_g5_index in range(len(topic_data_dict["/can0/received_msg"]) - 1):
            time_g5_current = time_g5_index

            for time_q4_index in range (len(topic_data_dict["/can2/received_msg"])):

                if (topic_data_dict["/can0/received_msg"][time_g5_current] < topic_data_dict["/can2/received_msg"][time_q4_index] < topic_data_dict["/can0/received_msg"][time_g5_current + 1]):
                    
                    # 怕第一筆資料找到G5前的資料
                    if (time_q4_current == 0):
                        time_q4_current = time_q4_index

                    if (abs(topic_data_dict["/can2/received_msg"][time_q4_index] - topic_data_dict["/can0/received_msg"][time_g5_current])
                        < abs(topic_data_dict["/can2/received_msg"][time_q4_current] - topic_data_dict["/can0/received_msg"][time_g5_current])):
                        time_q4_current = time_q4_index

            for time_image_index in range (len(topic_data_dict["/cme_cam"])):
                
                # Q4 找離 G5 後最近的
                if (topic_data_dict["/can0/received_msg"][time_g5_current] < topic_data_dict["/cme_cam"][time_image_index] < topic_data_dict["/can0/received_msg"][time_g5_current + 1]):
                    
                    if (time_image_current == 0):
                        time_image_current = time_image_index
                    
                    if (abs(topic_data_dict["/cme_cam"][time_image_index] - topic_data_dict["/can0/received_msg"][time_g5_current])
                        < abs(topic_data_dict["/cme_cam"][time_image_current] - topic_data_dict["/can0/received_msg"][time_g5_current])):
                        time_image_current = time_image_index

            for time_imu_index in range (len(topic_data_dict["/imu"])):

                # Imu 找離 G5 後最近的
                if (topic_data_dict["/can0/received_msg"][time_g5_current] < topic_data_dict["/imu"][time_imu_index] < topic_data_dict["/can0/received_msg"][time_g5_current + 1]):
                    
                    if (time_imu_current == 0):
                        time_imu_current = time_imu_index
                    
                    if (abs(topic_data_dict["/imu"][time_imu_index] - topic_data_dict["/can0/received_msg"][time_g5_current])
                        < abs(topic_data_dict["/imu"][time_imu_current] - topic_data_dict["/can0/received_msg"][time_g5_current])):
                        time_imu_current = time_imu_index

            temp=0
            for time_lidar_index in range (len(topic_data_dict["/rslidar_points"])):
                
                # Lidar 找離 G5 開頭最近的
                if (topic_data_dict["/rslidar_points"][time_lidar_index] < topic_data_dict["/can0/received_msg"][time_g5_current + 1]):
                    
                    if (abs(topic_data_dict["/rslidar_points"][time_lidar_index] - topic_data_dict["/can0/received_msg"][time_g5_current])
                        < abs(topic_data_dict["/rslidar_points"][time_lidar_current] - topic_data_dict["/can0/received_msg"][time_g5_current])):
                        time_lidar_current = time_lidar_index

            time_topic_arranged.append((topic_data_dict['/can0/received_msg'][time_g5_current],
                                       topic_data_dict['/can2/received_msg'][time_q4_current],
                                       topic_data_dict['/cme_cam'][time_image_current],
                                       topic_data_dict['/imu'][time_imu_current],
                                       topic_data_dict['/rslidar_points'][time_lidar_current]))
            
            """test arranged time stamp output"""
            # print(f"g5 time => {topic_data_dict['/can0/received_msg'][time_g5_current]}")
            # print(f"q4 time => {topic_data_dict['/can2/received_msg'][time_q4_current]}")
            # print(f"image time => {topic_data_dict['/cme_cam'][time_image_current]}")
            # print(f"imu time => {topic_data_dict['/imu'][time_imu_current]}")
            # print(f"lidar time => {topic_data_dict['/rslidar_points'][time_lidar_current]}")
            # print("--------------------------------------------------------------------")


        # 從 bag 讀取 time_topic_arranged timestamp 的資料
        for topic_sets in range (len(time_topic_arranged)):

            # SETTING OUTPUT PATH
            xml_output_path = xml_path
            img_output_path = img_path

            # Flags
            g5_start = False
            q4_start = False

            Mobileye_Q4_line_type = ["Undecided", "Solid", "Dahsed", "Double Line Marking", "Basis of Track for Temporary Shifts", "Deceleration", "High Occupancy Vehicle Lane"]
            Mobileye_Q4_object_class_name = ["car", "truck", "bike", "bicycle", "pedestrian", "general_object", "unknown", "uncertain_vcl"]
            IVA_G5_line_type = ["Undecided", "Single Solid", "Double Solid", "Single Dashed", "Road Edge"]
            IVA_G5_object_class_name = ["background", "car_big", "car_small", "motor", "bike", "person", "e_motor_pd", "e_bike_pd", "e_car_big", "e_car_small", "e_motor", "e_bike"]
            
            # _xml is root
            _xml = Node("annotation")
            IVA_G5_xml = Node("IVA_G5")
            Mobileye_Q4_xml = Node("Mobileye_Q4")
            Imu_xml = Node("Imu")
            Lidar_xml = Node("Lidar")

            _xml.add(IVA_G5_xml)
            _xml.add(Mobileye_Q4_xml)
            _xml.add(Imu_xml)
            _xml.add(Lidar_xml)

            # OPEN BAG
            try:
                bag = rosbag.Bag(bag_file, "r")
            except rosbag.bag.ROSBagException as e:
                print(f"Error opening ROS Bag file: {e}")
                continue
            
            # print("====================================================================")
            # IVA G5
            messages = bag.read_messages(topics="/can0/received_msg")
            for topic, msg, t in messages:

                timefloat = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timefloat)
                
                if (timefloat == time_topic_arranged[topic_sets][topic_to_read.index("/can0/received_msg")]):

                    g5_start = True

                    # print(topic, timestr)
                    # print(f"IVA G5 Start => {timestr}")

                    IVA_G5_timestamp_xml = Node("timestamp", timestr)
                    IVA_G5_lanes_xml = Node("lanes")
                    IVA_G5_objects_xml = Node("objects")

                    IVA_G5_LH_left_xml = Node("lane")
                    IVA_G5_LH_right_xml = Node("lane")
                    IVA_G5_LA_left_xml = Node("lane")
                    IVA_G5_LA_right_xml = Node("lane")

                    IVA_G5_xml.add(IVA_G5_timestamp_xml)
                    IVA_G5_xml.add(IVA_G5_lanes_xml)
                    IVA_G5_xml.add(IVA_G5_objects_xml)

                    IVA_G5_lanes_xml.add(IVA_G5_LH_left_xml)
                    IVA_G5_lanes_xml.add(IVA_G5_LH_right_xml)
                    IVA_G5_lanes_xml.add(IVA_G5_LA_left_xml)
                    IVA_G5_lanes_xml.add(IVA_G5_LA_right_xml)

                if (g5_start):
                    if ((hex(msg.id) == "0x4e1") and ((timefloat - time_topic_arranged[topic_sets][topic_to_read.index("/can0/received_msg")]) > 0.075)):
                        g5_start = False

                        # print(f"IVA G5 Stop => {timestr}")
                    
                        IVA_G5_LH_left_xml.add(Node("type", "host_left"))
                        IVA_G5_LH_left_xml.add(IVA_G5_LH_left_line_type_xml)
                        IVA_G5_LH_left_xml.add(IVA_G5_LH_left_curvature_xml)
                        IVA_G5_LH_left_xml.add(IVA_G5_LH_left_distance_xml)

                        IVA_G5_LH_right_xml.add(Node("type", "host_right"))
                        IVA_G5_LH_right_xml.add(IVA_G5_LH_right_line_type_xml)
                        IVA_G5_LH_right_xml.add(IVA_G5_LH_right_curvature_xml)
                        IVA_G5_LH_right_xml.add(IVA_G5_LH_right_distance_xml)

                        IVA_G5_LA_left_xml.add(Node("type", "adjacent_left"))
                        IVA_G5_LA_left_xml.add(IVA_G5_LA_left_line_type_xml)
                        IVA_G5_LA_left_xml.add(IVA_G5_LA_left_curvature_xml)
                        IVA_G5_LA_left_xml.add(IVA_G5_LA_left_distance_xml)

                        IVA_G5_LA_right_xml.add(Node("type", "adjacent_right"))
                        IVA_G5_LA_right_xml.add(IVA_G5_LA_right_line_type_xml)
                        IVA_G5_LA_right_xml.add(IVA_G5_LA_right_curvature_xml)
                        IVA_G5_LA_right_xml.add(IVA_G5_LA_right_distance_xml)

                    else:
                        data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

                        # Lane Host Left
                        if (hex(msg.id) == "0x4e2"):
                            quality, exist, G5_L1 = IVA_lanecurve_2901379(data_str, flag='1')

                            IVA_G5_LH_left_curvature_xml = Node("curvature")

                            for i in range (4):
                                IVA_G5_LH_left_curvature_xml.add(Node(f"C{i}", str(np.round(G5_L1[i], 4))))

                        elif (hex(msg.id) == "0x4e9"):
                            g5l1_info = IVA_lanecurve_2902468(data_str)
                        
                        # Lane Host Right
                        elif (hex(msg.id) == "0x4e3"):
                            quality, exist, G5_R1 = IVA_lanecurve_2901379(data_str, flag='3')
                        
                            IVA_G5_LH_right_curvature_xml = Node("curvature")

                            for i in range (4):
                                IVA_G5_LH_right_curvature_xml.add(Node(f"C{i}", str(np.round(G5_R1[i],4))))

                        elif (hex(msg.id) == "0x4ea"):
                            g5r1_info = IVA_lanecurve_2902468(data_str)
                        
                        # Lane Adjacent Left
                        elif (hex(msg.id) == "0x4e4"):
                            quality, exist,G5_L2 = IVA_lanecurve_2901379(data_str, flag='7')

                            IVA_G5_LA_left_curvature_xml = Node("curvature")

                            for i in range (4):
                                IVA_G5_LA_left_curvature_xml.add(Node(f"C{i}", str(np.round(G5_L2[i],4))))

                        elif (hex(msg.id) == "0x4eb"):
                            g5l2_info = IVA_lanecurve_2902468(data_str)
                        
                        # Lane Adjacent Right
                        elif (hex(msg.id) == "0x4e5"):
                            quality, exist,G5_R2 = IVA_lanecurve_2901379(data_str, flag='9')

                            IVA_G5_LA_right_curvature_xml = Node("curvature")

                            for i in range (4):
                                IVA_G5_LA_right_curvature_xml.add(Node(f"C{i}", str(np.round(G5_R2[i],4))))

                        elif (hex(msg.id) == "0x4ec"):
                            g5r2_info = IVA_lanecurve_2902468(data_str)
                        
                        # Lane Overview
                        elif (hex(msg.id) == "0x4e6"):
                            dis_L_1st, dis_R_1st, type_L_1st, type_R_1st, dis_L_2nd, dis_R_2nd, type_L_2nd, type_R_2nd = IVA_Lane_Distance_decode(data_str)
                            
                            IVA_G5_LH_left_line_type_xml = Node("line_type", IVA_G5_line_type[type_L_1st])
                            IVA_G5_LH_right_line_type_xml = Node("line_type", IVA_G5_line_type[type_R_1st])
                            IVA_G5_LA_left_line_type_xml = Node("line_type", IVA_G5_line_type[type_L_2nd])
                            IVA_G5_LA_right_line_type_xml = Node("line_type", IVA_G5_line_type[type_R_2nd])
                            IVA_G5_LH_left_distance_xml = Node("distance", dis_L_1st)
                            IVA_G5_LH_right_distance_xml = Node("distance", dis_R_1st)
                            IVA_G5_LA_left_distance_xml = Node("distance", dis_L_2nd)
                            IVA_G5_LA_right_distance_xml = Node("distance", dis_R_2nd)

                        # Object Overview
                        elif (hex(msg.id) == "0x4e8"):
                            G5obnum, G5lm = IVA_obj_290D(data_str)

                        elif (hex(msg.id) == "0x4e7"):
                            OBJ_id, OBJ_class, OBJ_conf, OBJ_pos_x, OBJ_pos_y, OBJ_vel_x, OBJ_vel_y = IVA_obj_290B(data_str)

                            IVA_G5_object_xml = Node("object")
                            IVA_G5_objects_xml.add(IVA_G5_object_xml)

                            IVA_G5_object_xml.add(Node("id", f"{OBJ_id}"))
                            IVA_G5_object_xml.add(Node("class", f"{IVA_G5_object_class_name[OBJ_class]}"))
                            IVA_G5_object_xml.add(Node("x", f"{OBJ_pos_x:.3f}"))
                            IVA_G5_object_xml.add(Node("y", f"{OBJ_pos_y:.3f}"))
            
            # Mobileye Q4
            messages = bag.read_messages(topics="/can2/received_msg")
            for topic, msg, t in messages:

                timefloat = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timefloat)
                
                if (timefloat == time_topic_arranged[topic_sets][topic_to_read.index("/can2/received_msg")]):

                    q4_start = True

                    # print(topic, timestr)
                    # print(f"Mobileye Q4 Start => {timestr}")

                    Mobileye_Q4_timestamp_xml = Node("timestamp", timestr)
                    Mobileye_Q4_lanes_xml = Node("lanes")
                    Mobileye_Q4_objects_xml = Node("objects")
                    
                    Mobileye_Q4_xml.add(Mobileye_Q4_timestamp_xml)
                    Mobileye_Q4_xml.add(Mobileye_Q4_lanes_xml)
                    Mobileye_Q4_xml.add(Mobileye_Q4_objects_xml)

                if (q4_start):
                    if (hex(msg.id) == "0x700" and ((timefloat - time_topic_arranged[topic_sets][topic_to_read.index("/can2/received_msg")]) > 0)):
                        q4_start = False

                        # print(f"Mobileye Q4 Stop => {timestr}")

                    else:
                        data_str = prettyhex2(msg.dlc, msg.data, '-').split('-')

                        if (hex(msg.id) == "0x770"):
                            # 主車道第一筆資料
                            [Q4L_conf, Q4L_type, Q4L_side, Q4L_start, Q4L_end] = Q4_770_772(data_str)
                            Mob_L_range = int(Q4L_end - Q4L_start)
                            
                            Mobileye_Q4_lane_xml = Node("lane")
                            Mobileye_Q4_LH_left_distance_xml = Node("distance", f"{Mob_L_range}")

                            Mobileye_Q4_lanes_xml.add(Mobileye_Q4_lane_xml)
                            Mobileye_Q4_lane_xml.add(Node("type", "host_left"))

                            # 例外處理 車道線樣式
                            if (Q4L_type >= len(Mobileye_Q4_line_type)):
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[0]))
                            else:
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[Q4L_type]))

                        elif (hex(msg.id) == "0x771"):
                            Mob_L_C_L = Q4_771_773(data_str)

                            Mobileye_Q4_LH_left_curvature_xml = Node("curvature")
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LH_left_curvature_xml)
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LH_left_distance_xml)
                            for i in range (4):
                                Mobileye_Q4_LH_left_curvature_xml.add(Node(f"C{i}", str(np.round(Mob_L_C_L[i],4))))

                        elif (hex(msg.id) == "0x772"):
                            [Q4R_conf, Q4R_type, Q4R_side, Q4R_start, Q4R_end] = Q4_770_772(data_str)
                            Mob_R_range = int(Q4R_end-Q4R_start)
                            
                            Mobileye_Q4_lane_xml = Node("lane")
                            Mobileye_Q4_LH_right_distance_xml = Node("distance", f"{Mob_R_range}")

                            Mobileye_Q4_lanes_xml.add(Mobileye_Q4_lane_xml)
                            Mobileye_Q4_lane_xml.add(Node("type", "host_right"))

                            # 例外處理 車道線樣式
                            if (Q4R_type >= len(Mobileye_Q4_line_type)):
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[0]))
                            else:
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[Q4R_type]))
                        
                        elif (hex(msg.id) == "0x773"):
                            Mob_R_C_L = Q4_771_773(data_str)

                            Mobileye_Q4_LH_right_curvature_xml = Node("curvature")
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LH_right_curvature_xml)
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LH_right_distance_xml)
                            for i in range (4):
                                Mobileye_Q4_LH_right_curvature_xml.add(Node(f"C{i}", str(np.round(Mob_R_C_L[i],4))))

                        elif (hex(msg.id) == "0x784"):
                            # 相鄰車道第一筆資料
                            [adj_typ, adj_conf, adj_start, adj_end, adj_role] = Q4_78468A(data_str)
                            range0 = int(adj_end-adj_start)

                            Mobileye_Q4_lane_xml = Node("lane")
                            Mobileye_Q4_LA_left_distance_xml = Node("distance", f"{range0}")

                            Mobileye_Q4_lanes_xml.add(Mobileye_Q4_lane_xml)
                            Mobileye_Q4_lane_xml.add(Node("type", "adjacent_left"))

                            # 例外處理 車道線樣式
                            if (adj_typ >= len(Mobileye_Q4_line_type)):
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[0]))
                            else:
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[adj_typ]))

                        elif (hex(msg.id) == "0x785"):
                            adj = Q4_78579B(data_str)
                            f_adj=1

                            Mobileye_Q4_LA_left_curvature_xml = Node("curvature")
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LA_left_curvature_xml)
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LA_left_distance_xml)
                            for i in range (4):
                                Mobileye_Q4_LA_left_curvature_xml.add(Node(f"C{i}", str(np.round(adj[i],4))))

                        elif (hex(msg.id) == "0x786"):
                            [adj_typ1, adj_conf1, adj_start1, adj_end1, adj_role1] = Q4_78468A(data_str)
                            range1 = int(adj_end1-adj_start1)

                            Mobileye_Q4_lane_xml = Node("lane")
                            Mobileye_Q4_LA_right_distance_xml = Node("distance", f"{range1}")

                            Mobileye_Q4_lanes_xml.add(Mobileye_Q4_lane_xml)
                            Mobileye_Q4_lane_xml.add(Node("type", "adjacent_right"))

                            # 例外處理 車道線樣式
                            if (adj_typ1 >= len(Mobileye_Q4_line_type)):
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[0]))
                            else:
                                Mobileye_Q4_lane_xml.add(Node("line_type", Mobileye_Q4_line_type[adj_typ1]))

                        elif (hex(msg.id) == "0x787"):
                            adj1 = Q4_78579B(data_str)
                            f_adj1=1

                            Mobileye_Q4_LA_right_curvature_xml = Node("curvature")
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LA_right_curvature_xml)
                            Mobileye_Q4_lane_xml.add(Mobileye_Q4_LA_right_distance_xml)
                            for i in range (4):
                                Mobileye_Q4_LA_right_curvature_xml.add(Node(f"C{i}", str(np.round(adj1[i],4))))

                        elif (hex(msg.id) == "0x788"):
                            [adj_typ2, adj_conf2, adj_start2, adj_end2, adj_role2] = Q4_78468A(data_str)
                            range2 = int(adj_end2-adj_start2)

                        elif (hex(msg.id) == "0x789"):
                            adj2 = Q4_78579B(data_str)
                            f_adj2=1

                        elif (hex(msg.id) == "0x78a"):
                            [adj_typ3, adj_conf3, adj_start3, adj_end3, adj_role3] = Q4_78468A(data_str)
                            range3 = int(adj_end3-adj_start3)

                        elif (hex(msg.id) == "0x78b"):
                            adj3 = Q4_78579B(data_str)
                            f_adj3=1

                        elif (hex(msg.id) == "0x541"):
                            # "CIPV" 通常指的是 "Critical Instant of Possible Collision Vehicle"，也就是可能發生碰撞的關鍵瞬間車輛。
                            [Mob_Num_Objs, CIPV, CIPVlost, VDacc] = Q4_541(data_str)
                            Mobileye_Q4_Num_Objs_count = Mob_Num_Objs

                        elif (hex(msg.id) in ["0x500", "0x503", "0x506", "0x509", "0x50c", "0x50f", "0x512", "0x515", "0x518", "0x51b", "0x51e", "0x521"]):
                            [iddd, Obj_class, OBJ_W, OBJ_L, Relative_Long_Velocity, OBJ_Lane_Ass, Relative_Lat_Velocity] = Q4_500(data_str)

                            if (Mobileye_Q4_Num_Objs_count):
                                Mobileye_Q4_object_xml = Node("object")
                                Mobileye_Q4_objects_xml.add(Mobileye_Q4_object_xml)

                                Mobileye_Q4_object_xml.add(Node("id", f"{iddd}"))
                                Mobileye_Q4_object_xml.add(Node("class", Mobileye_Q4_object_class_name[Obj_class]))

                        elif (hex(msg.id) in ["0x501", "0x504", "0x507", "0x50a", "0x50d", "0x50g", "0x510", "0x513", "0x516", "0x519", "0x51c", "0x51f", "0x522"]):
                            [Absolute_Long_Acc, Long_Distance_Acc, Lateral_Distance] = Q4_501(data_str)
                            
                            if (Mobileye_Q4_Num_Objs_count):
                                Mobileye_Q4_object_xml.add(Node("x", f"{Lateral_Distance:.3f}"))
                                Mobileye_Q4_object_xml.add(Node("y", f"{Long_Distance_Acc:.3f}"))
                            
                        elif (hex(msg.id) in ["0x502", "0x505", "0x508", "0x50b", "0x50e", "0x511", "0x514", "0x517", "0x51a", "0x51d", "0x520", "0x523"]):
                            [Absolute_Speed, OBJ_Motion_Status, OBJ_Motion_Category, Brake_Light, Turn_Indicator_Right, Turn_Indicator_Left, Light_indicator_validity, OBJ_Angle_Mid, OBJ_Angle_Rate] = Q4_502(data_str)

                            if (Mobileye_Q4_Num_Objs_count):
                                Mobileye_Q4_Num_Objs_count -= 1

                        elif (hex(msg.id) in ["0x720", "0x721", "0x722", "0x723", "0x724", "0x725", "0x726"]):
                            Mob_tsr0 = Q4_720_6(data_str)

                        elif (hex(msg.id) == "0x727"):
                            Mob_Sign_type = Q4_727(data_str)

            # Image
            messages = bag.read_messages(topics="/cme_cam")
            for topic, msg, t in messages:

                timefloat = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timefloat)
                
                if (timefloat == time_topic_arranged[topic_sets][topic_to_read.index("/cme_cam")]):

                    # print(topic, timestr)
                    # print(f"Image Start => {timestr}")

                    if (topic == "/cme_cam"):
                        # SET Image timestamp as xml output name
                        xml_output_name = os.path.join(xml_output_path, timestr)
                        img_output_name = os.path.join(img_output_path, timestr)
                        img = np.frombuffer(msg.data, dtype=np.uint8)
                        img = img.reshape(msg.height, msg.width,-1)
                        
                        
                        Image_xml = Node("Image")
                        Image_timestampe_xml = Node("timestamp", timestr)

                        _xml.add(Image_xml)
                        Image_xml.add(Image_timestampe_xml)
                        Image_xml.add(Node("format", "png"))
                        Image_xml.add(Node("width", f"{msg.width}"))
                        Image_xml.add(Node("height", f"{msg.height}"))


                        # OUTPUT Image
                        cv2.imwrite(f"{img_output_name}.png", img)
                    else:
                        print("expect topic /cme_cam, but actually ", topic)

            # Imu
            messages = bag.read_messages(topics="/imu")
            for topic, msg, t in messages:

                timefloat = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timefloat)
                
                if (timefloat == time_topic_arranged[topic_sets][topic_to_read.index("/imu")]):

                    # print(topic, timestr)
                    # print(f"Imu Start => {timestr}")
                    
                    if (topic == "/imu"):
                        Imu_timestamp_xml = Node("timestamp", timestr)
                        Imu_angular_velocity_xml = Node("angular_velocity")
                        Imu_linear_acceleration_xml = Node("linear_acceleration")

                        Imu_xml.add(Imu_timestamp_xml)
                        Imu_xml.add(Imu_angular_velocity_xml)
                        Imu_xml.add(Imu_linear_acceleration_xml)
                        Imu_angular_velocity_xml.add(Node("x", f"{msg.angular_velocity.x}"))
                        Imu_angular_velocity_xml.add(Node("y", f"{msg.angular_velocity.y}"))
                        Imu_angular_velocity_xml.add(Node("z", f"{msg.angular_velocity.z}"))
                        Imu_linear_acceleration_xml.add(Node("x", f"{msg.linear_acceleration.x}"))
                        Imu_linear_acceleration_xml.add(Node("y", f"{msg.linear_acceleration.y}"))
                        Imu_linear_acceleration_xml.add(Node("z", f"{msg.linear_acceleration.z}"))
                    else:
                        print("expect topic imu, but actually ", topic)

            # Lidar
            messages = bag.read_messages(topics="/rslidar_points")
            for topic, msg, t in messages:

                timefloat = msg.header.stamp.to_sec()
                timestr = "{:.6f}".format(timefloat)
                
                if (timefloat == time_topic_arranged[topic_sets][topic_to_read.index("/rslidar_points")]):

                    # print(topic, timestr)
                    # print(f"Lidar Start => {timestr}")
                    
                    Lidar_timestamp_xml = Node("timestamp", timestr)
                    Lidar_point_cloud_xml = Node("point_cloud")

                    Lidar_xml.add(Lidar_timestamp_xml)
                    Lidar_xml.add(Lidar_point_cloud_xml)

                    gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                    for p in gen:

                        # 過濾點雲 可能會沒有輸出
                        # if p[0] == 0 and p[1] == 0 and p[2] == 0:
                        #     continue
                        # elif p[2] < 0 or p[2] > 0.5:
                        #     continue
                        # elif p[0] > 10:
                        #     continue
                        # elif p[0] > 7.18 and p[0] < 14.36:
                        #     long_side  = 14.36 - p[0]
                        #     short_side = math.tan(19.2) * long_side * -1
                        #     if p[1] < short_side:
                        #         continue
                        # elif p[0] >= 14.36:
                        #     long_side  = p[0] - 14.36
                        #     short_side = math.tan(19.2) * long_side
                        #     if p[1] <= short_side:
                        #         continue
                    
                        Lidar_point_xml = Node("point")
                        Lidar_point_cloud_xml.add(Lidar_point_xml)

                        Lidar_point_xml.add(Node("x", f"{p[0]}")) # f"{p[0]:.3f}"
                        Lidar_point_xml.add(Node("y", f"{p[1]}"))
                        Lidar_point_xml.add(Node("z", f"{p[2]}"))
                        Lidar_point_xml.add(Node("intensity", f"{p[3]}"))
            
            # XML WRITE
            ff = open(f"{xml_output_name}.xml", 'w')
            _xml.writexml(ff)
            ff.close()

            # CLOSE BAG
            bag.close()

        # Move Bag into file
        shutil.move(bag_file, output_path)

        print(f"Finished\t{files} !!!!!!!")