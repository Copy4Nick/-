import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as ImagePIL
import colorsys   
from clover.srv import SetLEDEffect

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


bridge = CvBridge()

color = 'undefined'

color_pub = rospy.Publisher('color', Image, queue_size=1)
color2_pub =rospy.Publisher('color2', Image, queue_size=1)

imgg = cv2.imread('test17.jpg')
imgg_hsv = cv2.cvtColor(imgg, cv2.COLOR_BGR2HSV)

height, width, channels = imgg.shape
temp_height = height//2
temp_width = width//2
h,s,v = 0,0,0
for i in range(10):
    for j in range(10):
        h += imgg_hsv[temp_height-i][temp_width-j][0]
        s += imgg_hsv[temp_height-i][temp_width-j][1]
        v += imgg_hsv[temp_height-i][temp_width-j][2]
        
K = 0.14

h_spread = 8
s_spread = 20
v_spread = 70

h = int(h)//100
s = int(s)//100
v = int(v)//100
# print(height, width)
print(h,s,v)

h1 = h - h_spread
s1 = s - s_spread
v1 = v - v_spread

h2 = h + h_spread
s2 = s + s_spread
v2 = v + v_spread

if h1<0 : h1=0
if s1<0 : s1=0
if v1<0 : v1=0

if h2>179 : h2= 179
if s2>255 : s2=255
if v2>255 : v2= 255




clr_low = (h1, s1, v1)
clr_high= (h2, s2, v2)

center_pixel = []

def image_callback(data):
    global color
    global h, s, v
    global clr_low, clr_high
    global center_pixel

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)#[118:119,158:159]
    center_pixel = img_hsv[119][159]
    #print("hsv global", h, s, v)

    color_thresh = cv2.inRange(img_hsv, clr_low,clr_high)
    color2_pub.publish(bridge.cv2_to_imgmsg(color_thresh, 'mono8'))
    moments = cv2.moments(color_thresh)
    #print(moments["m00"])
    if moments["m00"] > 16320:
            cnt_x = int(moments["m10"] / moments["m00"])
            cnt_y = int(moments["m01"] / moments["m00"])
            cv_image = cv2.circle(cv_image, (cnt_x, cnt_y), 10, (0, 255, 0), 3)
            #print(cnt_x, cnt_y) 
            delta_y = -K * 2.0 * (cnt_x - 159) / data.height
            delta_x = K * 2.0 * (119 - cnt_y) / data.height
            #print(delta_x, delta_y)
            navigate(x = delta_x, y = delta_y, frame_id = 'setpoint')

    color_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
# set_effect(r=0, g=0, b=0)
# rospy.sleep(2)
navigate(x=0,y=0,z=1.3, speed=1, frame_id='body', auto_arm=True)

# rospy.sleep(3)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

while not rospy.is_shutdown():
    telemetry = get_telemetry() 
    print(telemetry.x, telemetry.y, telemetry.z)
    rospy.sleep(1)
rospy.spin()