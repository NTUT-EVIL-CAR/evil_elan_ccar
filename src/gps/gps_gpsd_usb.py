import rospy
from sensor_msgs.msg import NavSatFix
import gps

def gps_callback(gps_data):
    rospy.loginfo("Latitude: {}, Longitude: {}".format(gps_data.latitude, gps_data.longitude))

def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_listener()
    except rospy.ROSInterruptException:
        pass
