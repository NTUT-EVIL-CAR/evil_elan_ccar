import cv2
import os

def images_to_video(image_folder, video_name, fps):
    # 获取文件夹中所有以 .jpg 结尾的文件
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    
    # 按照文件名排序
    images.sort()  # 这里会按照文件名的字母顺序排序
    
    # 读取第一张图片，获取尺寸信息
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    # 创建视频写入器
    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    # 逐一将图片写入视频
    for image in images:
        img_path = os.path.join(image_folder, image)
        frame = cv2.imread(img_path)
        video.write(frame)

    # 释放资源
    cv2.destroyAllWindows()
    video.release()

# 指定图片文件夹路径、输出视频名称和帧率
image_folder = './image'
video_name = 'output.mp4'
fps = 10  # 视频帧率，每秒播放的帧数

# 调用函数生成视频
images_to_video(image_folder, video_name, fps)
