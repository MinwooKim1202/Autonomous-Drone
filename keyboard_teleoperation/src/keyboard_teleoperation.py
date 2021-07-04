#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, PoseStamped, Twist
import sys, select, termios, tty

LINEAR_MAX__VEL = 30
ANGULAR_MAX_VEL = 30
LINEAR_MIN__VEL = -30
ANGULAR_MIN_VEL = -30


msg = """
Control Your Quadcopter!
---------------------------
Moving around:
q w e
a s d
z x c

w/s : increase/decrease linear velocity X
a/d : increase/decrease linear velocity Y
q/e : increase/decrease angular velocity Z
z/c : increase/decrease linear velocity Z

space key, x : Hovering

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(linear_velocity_X, linear_velocity_Y, linear_velocity_Z, angular_velocity_Z):
    return "currently:\tlinear velocity X %s\t linear velocity Y \
    %s\tlinear velocity Z %s\t angular velocity Z %s " % (linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z)

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def checkLINEARLimitVelocity(vel):
    vel = constrain(vel, LINEAR_MIN__VEL, LINEAR_MAX__VEL)
    return vel

def checkANGULARLimitVelocity(vel):
    vel = constrain(vel, ANGULAR_MIN_VEL, ANGULAR_MAX_VEL)
    return vel

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('drone_teleop',anonymous=True)

    teleop_twist = Twist()
 
    status = 0
    linear_velocity_X = 0
    linear_velocity_Y = 0
    linear_velocity_Z = 0
    angular_velocity_Z = 0

    try:
        print(msg)
        while True:
            key = getKey()
            if key == 'w' :
                linear_velocity_X +=1
                linear_velocity_X = checkLINEARLimitVelocity(linear_velocity_X)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 's' :
                linear_velocity_X -=1
                linear_velocity_X = checkLINEARLimitVelocity(linear_velocity_X)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'a' :
                linear_velocity_Y -=1
                linear_velocity_Y = checkLINEARLimitVelocity(linear_velocity_Y)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'd' :
                linear_velocity_Y +=1
                linear_velocity_Y = checkLINEARLimitVelocity(linear_velocity_Y)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'z' :
                linear_velocity_Z -=1
                linear_velocity_Z = checkLINEARLimitVelocity(linear_velocity_Z)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'c' :
                linear_velocity_Z +=1
                linear_velocity_Z = checkLINEARLimitVelocity(linear_velocity_Z)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'q' :
                angular_velocity_Z -=1
                angular_velocity_Z = checkANGULARLimitVelocity(angular_velocity_Z)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'e' :
                angular_velocity_Z +=1
                angular_velocity_Z = checkANGULARLimitVelocity(angular_velocity_Z)
                status = status + 1
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            elif key == 'x' :
                print (vels(linear_velocity_X,linear_velocity_Y, linear_velocity_Z, angular_velocity_Z))
            else:
                if (key == '\x03'):
                    print("end")
                    break

            if status == 20 :
                print(msg)
                status = 0

            teleop_twist.linear.x = linear_velocity_X
            teleop_twist.linear.y = linear_velocity_Y
            teleop_twist.linear.z = linear_velocity_Z
            teleop_twist.angular.z = angular_velocity_Z
            pub.publish(teleop_twist)
    
    except rospy.ROSInterruptException:
        pass

    finally:
        pub.publish(teleop_twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
