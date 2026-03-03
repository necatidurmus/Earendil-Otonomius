#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Leo Rover WASD Teleop
---------------------------
Movment:
   w
 a   d
   s

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

SPACE : force stop
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0.0

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = TeleopNode()

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                node.x = float(moveBindings[key][0])
                node.th = float(moveBindings[key][3])
            elif key in speedBindings.keys():
                node.speed = node.speed * speedBindings[key][0]
                node.turn = node.turn * speedBindings[key][1]
                print(f"Speed: {node.speed} | Turn: {node.turn}")
            elif key == ' ':
                node.x = 0.0
                node.th = 0.0
            elif key == '\x03':
                break
            else:
                node.x = 0.0
                node.th = 0.0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = node.x * node.speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = node.th * node.turn
            node.publisher_.publish(twist)

    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        node.publisher_.publish(twist)
        
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
