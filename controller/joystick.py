from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String
from tello_controller.msg import command
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

print("PRESS Q or ESC TO EXIT")

SPEED = 0.3

def sendRc(pitch, roll, yaw, altitude):
    msg1 = PoseStamped()
    msg1.pose.orientation.x = roll
    msg1.pose.orientation.y = pitch
    msg1.pose.orientation.z = 0
    msg1.pose.orientation.w = 0.997
    msg2 = TwistStamped()
    msg2.twist.angular.z = yaw #YAW
    msg2.twist.linear.z = altitude #ALTITUDE
    pub_pitchRoll.publish(msg1)
    pub_yawAltitude.publish(msg2)

def emergency():
    msg = command()
    msg.command = msg.EMERGENCY_STOP
    pub_highLevel.publish(msg)

def land():
    msg = command()
    msg.command = msg.LAND
    pub_highLevel.publish(msg)

def takeoff():
    msg = command()
    msg.command = msg.TAKE_OFF
    pub_highLevel.publish(msg)

def on_press(key):
    print('{0} pressed'.format(key))

def on_press(key):
    print('{0} pressed'.format(key))
    if str(key) == "'t'":
        takeoff()
    if str(key) == "'y'":
        land()
    if str(key) == "'e'":
        emergency()
    if (key == Key.up):
        sendRc(SPEED,0,0,0)
    elif (key == Key.down):
        sendRc(-SPEED,0,0,0)
    elif(key == Key.left):
        sendRc(0,-SPEED,0,0)
    elif(key == Key.right):
        sendRc(0,SPEED,0,0)
    elif(str(key) == "'w'"):
        sendRc(0,0,0,SPEED)
    elif(str(key) == "'s'"):
        sendRc(0,0,0,-SPEED)
    elif(str(key) == "'d'"):
        sendRc(0,0,SPEED,0)
    elif(str(key) == "'a'"):
        sendRc(0,0,-SPEED,0)
        
def on_release(key):
    string = "rc 0 0 0 0"
    if key == Key.esc:
        # Stop listener
        return False
    if (str(key) == "'q'"):
        return False

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    pub = rospy.Publisher('drone0/command', String, queue_size=10)
    pub_highLevel = rospy.Publisher('drone0/command/high_level', command, queue_size=10)
    pub_pitchRoll = rospy.Publisher('drone0/actuator_command/roll_pitch', PoseStamped, queue_size=10)
    pub_yawAltitude = rospy.Publisher('drone0/actuator_command/altitude_rate_yaw_rate', TwistStamped, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    listener.join()


