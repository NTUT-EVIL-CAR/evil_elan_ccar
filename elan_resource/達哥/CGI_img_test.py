import cv2
import requests
import time

IVA_IP = "192.168.100.1"
GET_IMG_CGI = "http://" + IVA_IP + "/cgi-bin/Config.cgi?action=get&property=image&value=cam0"
GET_IMG_URL = "http://" + IVA_IP + "/image_0.jpg"
FPS = 30


def get_img():

    video_cap = cv2.VideoCapture(GET_IMG_URL)
    res, img = video_cap.read()
    while not(res):
        res, img = video_cap.read()
    video_cap.release()
    
    return img
    
    
def main():
    fail_cnt = 0
    wait_sec = 1/FPS
    
    while (fail_cnt<20):
        start_t = time.time()
        response = requests.get(GET_IMG_CGI)  
        if (response.status_code == 200):
            fail_cnt = 0
            img  = get_img()
            cv2.imshow("img", img)
            cv2.waitKey(1)
            
        else:
            fail_cnt += 1

        end_t = time.time()
        sleep_t = wait_sec - (end_t - start_t) 
        if sleep_t < 0:
            sleep_t = 0
        elif sleep_t > wait_sec:
            sleep_t = wait_sec

        time.sleep(sleep_t) 
        
if __name__ == "__main__":
    main()