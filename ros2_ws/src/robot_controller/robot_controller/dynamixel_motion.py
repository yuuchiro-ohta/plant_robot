import robot_controller.dynamixel_module as motor
import time

initial_position =[280, 180, 180, 280, 180, 180, 280, 180, 180, 280, 180, 180]



def init_position():
    motor.dynamixel_position_rotate(3, 180)
    motor.dynamixel_position_rotate(6, 180)
    motor.dynamixel_position_rotate(9, 180)
    motor.dynamixel_position_rotate(12, 180)

    time.sleep(0.01)

    motor.dynamixel_position_rotate(1, 280)
    motor.dynamixel_position_rotate(4, 280)
    motor.dynamixel_position_rotate(7, 280)
    motor.dynamixel_position_rotate(10, 280)

    time.sleep(0.01)

    motor.dynamixel_position_rotate(2, 180)
    motor.dynamixel_position_rotate(5, 180)
    motor.dynamixel_position_rotate(8, 180)
    motor.dynamixel_position_rotate(11, 180)

def init_motion():
    motor.dynamixel_position_rotate(11, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(2, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(12, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(3, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(11, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(10, 280)
    time.sleep(0.01)    
    motor.dynamixel_position_rotate(2, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(1, 280)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(8, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(6, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(9, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 280)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(8, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 280)
    time.sleep(0.01)  

def forward_motion():
    motor.dynamixel_position_rotate(11, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(2, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(10, 320)
    time.sleep(0.01)    
    motor.dynamixel_position_rotate(1, 260)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(12, 140)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(3, 140)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(11, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(2, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(10, 280)
    time.sleep(0.01)    
    motor.dynamixel_position_rotate(1, 280)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(8, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 320)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 260)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(6, 220)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(9, 220)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 180)
    time.sleep(0.01)    
    motor.dynamixel_position_rotate(8, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 280)
    time.sleep(0.01)

def back_motion():
    motor.dynamixel_position_rotate(8, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 120)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(7, 320)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(9, 220)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(6, 220)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(8, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 180)
    time.sleep(0.01)    

    motor.dynamixel_position_rotate(4, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 280)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(2, 120)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(11, 120)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(1, 320)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(10, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(3, 140)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(12, 140)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(2, 180)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(11, 180)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(1, 280)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(10, 280)
    time.sleep(0.01)

def right_motion():
    motor.dynamixel_position_rotate(11, 120)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(2, 120)
    time.sleep(0.01)    

    motor.dynamixel_position_rotate(10, 320)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(1, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(12, 200)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(3, 200)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(11, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(2, 180)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(10, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(1, 280)
    time.sleep(0.01)  

    motor.dynamixel_position_rotate(8, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 120)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(7, 320)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(9, 160)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(6, 160)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(8, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(5, 180)
    time.sleep(0.01)    

    motor.dynamixel_position_rotate(7, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(4, 280)
    time.sleep(0.01)

def left_motion():
    motor.dynamixel_position_rotate(5, 120)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(8, 120)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(4, 320)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(6, 160)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(9, 160)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(5, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(8, 180)
    time.sleep(0.01)    

    motor.dynamixel_position_rotate(4, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(7, 280)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(2, 120)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(11, 120)
    time.sleep(0.01)    

    motor.dynamixel_position_rotate(1, 320)
    time.sleep(0.01)  
    motor.dynamixel_position_rotate(10, 260)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(3, 200)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(12, 200)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(2, 180)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(11, 180)
    time.sleep(0.01)

    motor.dynamixel_position_rotate(1, 280)
    time.sleep(0.01)
    motor.dynamixel_position_rotate(10, 280)
    time.sleep(0.01)  

if __name__ == "__main__":
    init_position()
    right_motion()



