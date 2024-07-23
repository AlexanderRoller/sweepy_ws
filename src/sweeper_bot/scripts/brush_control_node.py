import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Joy 
import RPi.GPIO as GPIO
import time 

small_relay_pin1 = 17 # for side brush 1 
small_relay_pin2 = 27 # for side brush 2 
small_relay_pin3 = 22 # for main brush 


# Pin setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(small_relay_pin1, GPIO.OUT)
GPIO.setup(small_relay_pin2, GPIO.OUT)
GPIO.setup(small_relay_pin3, GPIO.OUT)

class BrushControlNode(Node):
    def __init__(self):
        super().__init__('brush_control_node')
        self.subscription = self.create_subscription(
            Joy, 
            'joy',
            self.joy_callback,
            10
        )
        self.subscription 
        self.get_logger().info('Brush Control Node has been started.')

    def joy_callback(self, msg):
        if msg.buttons[0]:
            GPIO.output(small_relay_pin1, GPIO.LOW)
        else:
            GPIO.output(small_relay_pin1, GPIO.HIGH)

        if msg.buttons[1]:
            GPIO.output(small_relay_pin2, GPIO.LOW)
        else:
            GPIO.output(small_relay_pin2, GPIO.HIGH)

        if msg.buttons[2]:
            GPIO.output(small_relay_pin3, GPIO.LOW)
        else:
            GPIO.output(small_relay_pin3, GPIO.HIGH)

def main(args=None):
    rclpy.init(args=args)
    node=BrushControlNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#def turn_on_side_brush_1():
 #   GPIO.output(small_relay_pin1, GPIO.LOW)  #Assume relay is active when it is LOW 

#def turn_off_side_brush_1():
 #   GPIO.output(small_relay_pin1, GPIO.HIGH)

#def turn_on_side_brush_2():
  #  GPIO.output(small_relay_pin2, GPIO.LOW)  #Assume relay is active when it is LOW 

#def turn_off_side_brush_2():
 #   GPIO.output(small_relay_pin2, GPIO.HIGH)

#def turn_on_main_brush():
 #   GPIO.output(small_relay_pin3, GPIO.LOW)  #Assume relay is active when it is LOW 

#def turn_off_main_brush():
 #   GPIO.output(small_relay_pin3, GPIO.HIGH)

#try:
  #  while True:
 #       turn_on_side_brush_1()
   #     turn_on_side_brush_2()
    #    turn_on_main_brush()
     #   print("All brushes are ON")
      #  time.sleep(10) # Keep brushes on for 10 seconds 

       # turn_off_side_brush_1()
        #turn_off_side_brush_2()
        #turn_off_main_brush()
       # print("All brushes are OFF")
        #time.sleep(10) # Keep brushes off for 10 seconds 

#except KeyboardInterrupt:
 #   print("Exiting program")

#finally:
 #   GPIO.cleanup()




