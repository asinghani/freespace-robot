import time
from flask import Flask, request
import thread
import rospy
from geometry_msgs.msg import Twist

# Load HTML
html = "error"
with open("teleop_index.html", "r") as file:
    html = file.read()

speed = 0
steer = 0

app = Flask(__name__)

rospy.init_node('teleop_control_node', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0

@app.route("/get")
def getData():
    return "50,50,50,50"

@app.route("/set")
def setData():
    global speed, steer, vel_msg
    leftDrive = int(request.args.get("left"))
    rightDrive = int(request.args.get("right"))

    speed = (leftDrive + rightDrive) / 200.0
    steer = (leftDrive - rightDrive) / 500.0
    vel_msg.linear.x = speed
    vel_msg.angular.z = steer
    velocity_publisher.publish(vel_msg)
    return ""

@app.route("/drive")
def drivePage():
    return html

thread.start_new_thread(app.run, ())

