import cv2
import sys
import numpy as np
import os
import rosbag
import time
import re
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import matplotlib.lines as lines
import matplotlib as mpl

def get_curve(_C_L,_C_R,L_max_dis,R_max_dis):
    Y_L =  np.linspace(0,L_max_dis,720) # distance
    Y_R =  np.linspace(0,R_max_dis,720) # distance
    X_R,X_L =0,0
    for i in range(4):
        X_L = X_L + (_C_L[i]*(Y_L**i))
        X_R = X_R + (_C_R[i]*(Y_R**i))
        
    return X_L,X_R,Y_L,Y_R

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
 
def IVA_laneoverall_2905(num):
    # type and color
    L_dis1 = int(num[0],16)
    R_dis1 = int(num[1],16)
    L_dis2 = int(num[4],16)
    R_dis2 = int(num[5],16)
    return  L_dis1, R_dis1, L_dis2, R_dis2
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


bagpath = '/data2/G5Q4_bag/20230918'
res_path = 'save_0912/20230329'

if not os.path.exists(res_path):
    os.makedirs(res_path)

# tsr_enum = [(i) for i in range(14)]
# tsr_enum  += [(i) for i in range(28,42)]
# tsr_enum  += [(i) for i in range(100,130)]
# tsr_enum  += [(i) for i in range(171,177)]
# tsr_enum += [20, 50, 64, 65, 200, 201, 220, 221, 254, 255]
# tsr_enum.sort()

# import matplotlib.colors as mcolors

# mcolorslist = [name for name in mcolors.CSS4_COLORS]
# colorsets = {}
# for i in range(len(tsr_enum)):
#     colorsets[str(tsr_enum[i])] = mcolorslist[i]

# for files in os.listdir(bagpath):
for bb in range(1):
    files = '2023-09-15-16-33-32.bag'
    if files[-4:] == '.bag':
        print('analyze bag',files)
        # SETTINGS FOR SAVING OUTPUT VIDEO
        bag_file = os.path.join(bagpath,files)
        bagname = os.path.splitext(bag_file)[0]
        can1_file = bagname + '_can1.txt'
        res_path_img = os.path.join(res_path,files[:-4])
        out_file = os.path.join(res_path, os.path.split(bag_file)[1][:-4] + '_result.avi')

        cc=['C0','C1','C2','C3']
        roles=['None','LL','LR','RL','RR']
        CIPVlost_state=['No Loss','Lost Target FOV Out','Lost Target FOV in']
        VD_acc_state=['Free Space','Space not Free','Free Space Unknown']
        laneside=['Unknown','L','R']
        obj_ms = ['unknown','moving','stationary','stopped','moving Slowly','no','no','no','no']
        obj_ms_cate = ['undefined','passing', 'passingin','passingout','closecutin','movingin','movingout','crossing','ltap','rtap','moving','preceeding','oncoming','no','no','no']
        obj_lane_assign = ['nextL','L','Ego','R','nextR','notassign']
        offon = ['off','on']
        tf10 = ['false','true']
        Mob_out_message_str_time_L,Mob_out_message_str_time_R,Mob_out_message_str_L,Mob_out_message_str_R = '','','',''
        Mob_Num_Objs = 0
        linetype=['','-','--',':',':',':',':']
        fourcc_settings = cv2.VideoWriter_fourcc('M','J','P','G') 
        out_vid_dims = (1300//2, 1080//2)
        vid_dims = (600, 450)
        fps = 10  # adjust based on input video
        out = cv2.VideoWriter(out_file,
                              fourcc=fourcc_settings,
                              fps=fps,
                              frameSize=out_vid_dims,
                              isColor= True)
        # OPEN BAG
        bag = rosbag.Bag(bag_file, "r")
        print(bag.get_type_and_topic_info())

        # oldtopic='/camera_0/rgb/image_raw'
        newtopic='/cme_cam'
        messages = bag.read_messages(topics=[newtopic])  # Only get images data
        num_images = bag.get_message_count(topic_filters=[newtopic])
        fff=0
        g5t=[];imt=[]
        print('Load {} images from this bag file'.format(str(num_images)))

        len_img = num_images
        total_info = []
        for i in range(len_img):
            # READ NEXT MESSAGE IN BAG
            topic, msg, t = messages.__next__()
            img = np.fromstring(msg.data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width,-1)
            timeq = msg.header.stamp.to_sec() 
            timestr = "{:.6f}".format(timeq)

            # CONVERT MESSAGE TO A NUMPY ARRAY
            img_all = np.zeros((1080, 1300, 3), np.uint8)
            img_message = np.zeros((1080, 960, 3), np.uint8)
            img_all.fill(0)
            img_message.fill(0)

            timestr_s = get_two_float(timestr, 1)
            imt.append(float(timestr[-9:-3]))
            img = cv2.resize(img,vid_dims)
            img_all[300:750, 0:600] = img

            G5idall=[];G5clsall=[]
            G5xall=[];G5yall=[]
            G5_2901=0; flag_g5ok=0
            can1_f = open(can1_file,'r')
            can1_file_obj = can1_f.readlines()
            for l1,fileLine in enumerate(can1_file_obj):
                 can_message = fileLine.split(' ')
                 if (timeq < float(can_message[0])) and not flag_g5ok:
                    if len(can1_file_obj)> (l1+1):
                        can_message1 = can1_file_obj[l1+1].split(' ')
                        if (can_message[3].find('18fe5be0') > -1) and (can_message1[3].find('18fc2901') > -1):
                            if G5_2901 ==0:
                                G5_2901=1
                            else:
                                G5_2901=0
                                flag_g5ok=1
                    if G5_2901:
                        if can_message[3].find('18fc290d')> -1 :
                            timecan1=float(can_message[0])

                        if can_message[3].find('18fc2905') > -1:
                            data_str = can_message[9].split('-')
                            L_dis, R_dis, L_dis2, R_dis2 = IVA_laneoverall_2905(data_str)
                            out_message_str = can_message[0]
                            cv2.putText(img_message, out_message_str, (5, 30), cv2.FONT_HERSHEY_COMPLEX,1, (0, 255, 0), 1, cv2.LINE_AA)
                            out_message_str = 'IVA dist L : ' + str(L_dis) + ' R : ' + str(R_dis)
                            cv2.putText(img_message, out_message_str, (5, 55), cv2.FONT_HERSHEY_COMPLEX,1, (0, 255, 0), 1, cv2.LINE_AA)
        ###### L lane                
                        if can_message[3].find('18fc2901') > -1:
                            data_str = can_message[9].split('-')

                            timecan1=float(can_message[0])
                            timecan1str = "{:.6f}".format(timecan1)
                            print('can1 timestamp',float(timecan1str[-9:-1]))
                            
                            quality, exist,G5_L1 = IVA_lanecurve_2901379(data_str, flag='1')
                            
                            out_message_str = can_message[0]
                            cv2.putText(img_message, out_message_str, (5, 80), cv2.FONT_HERSHEY_COMPLEX,1, (255, 255, 0), 1, cv2.LINE_AA)
                            out_message_str = 'IVA L Sta: ' + str(exist) + ' Qult: ' + str(quality)
                            cv2.putText(img_message, out_message_str, (5, 105), cv2.FONT_HERSHEY_COMPLEX,1, (255, 255, 0), 1, cv2.LINE_AA)
                            
                            out_message_str = 'G5L : '
                            for ccc in range(4):
                                out_message_str += cc[ccc]+': '+str(np.round(G5_L1[ccc],4))+' '
                            cv2.putText(img_message, out_message_str, (5, 330), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (255, 255, 255), 1, cv2.LINE_AA)
                        if can_message[3].find('18fc2902') > -1:
                            data_str = can_message[9].split('-')
                            
                            g5l1_info = IVA_lanecurve_2902468(data_str)
                            out_message_str = 'G5 L1score: far: '+str(np.round(g5l1_info[0],3))+' mid: '+str(np.round(g5l1_info[1],3))+' close: '+str(np.round(g5l1_info[2],3))+' dis: '+str(np.round(g5l1_info[3],3))+' det: '+str(np.round(g5l1_info[4],3))
                            cv2.putText(img_message, out_message_str, (5, 805), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 36, 255), 1, cv2.LINE_AA)
                         
        ###### R lane            
                        if can_message[3].find('18fc2903') > -1:
                            data_str = can_message[9].split('-')
                            
                            quality, exist,G5_R1 = IVA_lanecurve_2901379(data_str, flag='3')
                            out_message_str = can_message[0]
                            cv2.putText(img_message, out_message_str, (5, 130), cv2.FONT_HERSHEY_COMPLEX,1, (0, 36, 255), 1, cv2.LINE_AA)
                            out_message_str = 'IVA R Sta: ' + str(exist) + ' Qult: ' + str(quality)
                            cv2.putText(img_message, out_message_str, (5, 155), cv2.FONT_HERSHEY_COMPLEX,1, (0, 36, 255), 1, cv2.LINE_AA)
                            out_message_str = 'G5R : '
                            for ccc in range(4):
                                out_message_str += cc[ccc]+': '+str(np.round(G5_R1[ccc],4))+' '
                            cv2.putText(img_message, out_message_str, (5, 355), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (255, 255, 255), 1, cv2.LINE_AA)
                        if can_message[3].find('18fc2904') > -1:
                            data_str = can_message[9].split('-')
                            
                            g5r1_info = IVA_lanecurve_2902468(data_str)
                            out_message_str = 'G5 R1score: far: '+str(np.round(g5r1_info[0],3))+' mid: '+str(np.round(g5r1_info[1],3))+' close: '+str(np.round(g5r1_info[2],3))+' dis: '+str(np.round(g5r1_info[3],3))+' det: '+str(np.round(g5r1_info[4],3))

                            cv2.putText(img_message, out_message_str, (5, 830), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 36, 255), 1, cv2.LINE_AA)
                         
         ###### L2 lane                
                        if can_message[3].find('18fc2907') > -1:
                            data_str = can_message[9].split('-')
                            
                            quality, exist,G5_L2 = IVA_lanecurve_2901379(data_str, flag='7')
                            out_message_str = 'G5L2 : '
                            for ccc in range(4):
                                out_message_str += cc[ccc]+': '+str(np.round(G5_L2[ccc],4))+' '
                            cv2.putText(img_message, out_message_str, (5, 505), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (255, 255, 255), 1, cv2.LINE_AA)
                        if can_message[3].find('18fc2906') > -1:
                            data_str = can_message[9].split('-')
                            
                            g5l2_info = IVA_lanecurve_2902468(data_str)
                            out_message_str = 'G5 L2score: far: '+str(np.round(g5l2_info[0],3))+' mid: '+str(np.round(g5l2_info[1],3))+' close: '+str(np.round(g5l2_info[2],3))+' dis: '+str(np.round(g5l2_info[3],3))+' det: '+str(np.round(g5l2_info[4],3))

                            cv2.putText(img_message, out_message_str, (5, 855), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 36, 255), 1, cv2.LINE_AA)
                         
        ###### R2 lane            
                        if can_message[3].find('18fc2909') > -1:
                            data_str = can_message[9].split('-')
                            
                            quality, exist,G5_R2 = IVA_lanecurve_2901379(data_str, flag='9')
                            out_message_str = 'G5R2 : '
                            for ccc in range(4):
                                out_message_str += cc[ccc]+': '+str(np.round(G5_R2[ccc],4))+' '
                            cv2.putText(img_message, out_message_str, (5, 530), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (255, 255, 255), 1, cv2.LINE_AA)
                        if can_message[3].find('18fc2908') > -1:
                            data_str = can_message[9].split('-')
                            
                            g5r2_info = IVA_lanecurve_2902468(data_str)
                            out_message_str = 'G5 R2score: far: '+str(np.round(g5r2_info[0],3))+' mid: '+str(np.round(g5r2_info[1],3))+' close: '+str(np.round(g5r2_info[2],3))+' dis: '+str(np.round(g5r2_info[3],3))+' det: '+str(np.round(g5r2_info[4],3))
                            cv2.putText(img_message, out_message_str, (5, 880), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 36, 255), 1, cv2.LINE_AA)
                         
                        if can_message[3].find('0x18fc290d')> -1:
                            data_str = can_message[9].split('-')
                            G5obnum, G5lm = IVA_obj_290D(data_str)
                        if can_message[3].find('0x18fc290b')> -1:
                            data_str = can_message[9].split('-')
                            G5ob = IVA_obj_290B(data_str)
                            G5idall.append(G5ob[0])
                            G5clsall.append(G5ob[1])
                            G5xall.append(G5ob[3])
                            G5yall.append(G5ob[4])

            can1_f.close()

      
            X_L,X_R,Y_L,Y_R = get_curve(G5_L1,G5_R1,L_dis,R_dis)
            X_L2,X_R2,Y_L2,Y_R2 = get_curve(G5_L2,G5_R2,L_dis,R_dis)
            plt.figure(figsize=(6,6))
            ax = plt.gca()
            ax.set_facecolor('black')
            plt.plot(X_L,Y_L,label='IVA',c='w',linewidth=5)
            # plt.plot(X_L2,Y_L2,label='IVA',c='w',linewidth=5)
            plt.plot(X_R,Y_R,label='IVA',c='w',linewidth=5)
            # plt.plot(X_R2,Y_R2,label='IVA',c='w',linewidth=5)
           
            plt.xlim(-10,10)
            plt.ylim(0,100)
            plt.grid()
            plt.xticks(np.arange(-10, 10, 1))
            plt.yticks(np.arange(0, 100, 10))
    #        plt.axis('equal')
            colors=['cyan','orange','g','orchid','y','pink','gray','r','r','r','r','r']
            colorsG5=['r','orange','cyan','pink','orchid','y','g','gray','orange','cyan','pink','orchid']
            def rott(box,theta,x,y):
                box_m=np.array(box)-np.repeat(np.array([[x,y]]),len(box),0)
                rotm=np.array([[np.cos(theta),-np.sin(theta)],
                               [np.sin(theta),np.cos(theta)]],np.float)
                newb=box_m.dot(rotm)+np.repeat(np.array([[x,y]]),len(box),0)
                return newb
           
            for j in range(len(G5idall)):
                if G5yall[j] <= 100:
                    plt.plot(G5xall[j],G5yall[j],'o',color='lightseagreen',markersize=7)
                    plt.text(G5xall[j]+0.4,G5yall[j]-2,str(G5idall[j])+'-'+str(G5lm[j]),fontsize=7,color='w')
            
            plt.savefig(os.path.join(res_path, 'lane.png'))
            plt.clf()
            plt.cla()
            plt.close('all')
            
            img_lane_tmp = np.zeros((600, 400, 3), np.uint8)
            img_lane_tmp = cv2.imread(os.path.join(res_path, 'lane.png'))
            img_lane = cv2.resize(img_lane_tmp,(700, 1080))
            img_all[0:1080, 600:1300] = img_lane
            cv2.imwrite(os.path.join(res_path, 'ori.png'),img_all)
            img_all = cv2.resize(img_all, out_vid_dims)
            out.write(img_all)  
                        
            # QUIT IF ESCAPE BUTTON PRESSED
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        out.release()


        cv2.destroyAllWindows()
        print("DONE!")

