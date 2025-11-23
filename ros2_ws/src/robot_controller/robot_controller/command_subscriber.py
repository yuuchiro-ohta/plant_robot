import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import robot_controller.dynamixel_motion as motion
import robot_controller.dynamixel_module as motor
import robot_controller.speak_and_listen as speak_and_listen
import time
import threading

initial_position = [280, 180, 180, 280, 180, 180, 280, 180, 180, 280, 180, 180]

class CommandSubscriberAction(Node):
    def __init__(self):
        super().__init__('command_subscriber_action')

        #ノードの生成
        self.subscription = self.create_subscription(
            String, 
            '/motion_command', 
            self.listener_callback, 
            10
        )
        self.current_thread = None
        self.get_logger().info("Motion controller started. Waiting for commands...")

        self.init_setting()

    def init_setting(self):
        for i, position in enumerate(initial_position, start=1):
            motor.dynamixel_lED_control(i, 1)
            motor.dynamixel_torque_control(i, 1)
            time.sleep(0.01)
        self.get_logger().info("Initialized positions")
            
    def listener_callback(self, msg):
        cmd = msg.data.lower()
        self.get_logger().info(f"Received command: {cmd}")

        # 前の動作を止める
        if self.current_thread and self.current_thread.is_alive():
            self.get_logger().warn("Stopping previous motion...")
            self.stop_motion = True
            self.current_thread.join()

        self.stop_motion = False

        if cmd == "init":
            self.start_motion(self.init_motion)            
        elif cmd == "forward":
            self.start_motion(self.forward_motion)
        elif cmd == "back":
            self.start_motion(self.back_motion)
        elif cmd == "left":
            self.start_motion(self.left_motion)
        elif cmd == "right":
            self.start_motion(self.right_motion)
        elif cmd == "talk":
            self.start_motion(self.talk_motion)
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    def start_motion(self, motion_func):
        self.current_thread = threading.Thread(target=motion_func)
        self.current_thread.start()
    
    # ==== モーション定義 ====
    def init_motion(self):
        self.stop_motion = False
        motion.init_motion()
        time.sleep(0.01)

    def forward_motion(self):
        while not self.stop_motion:
            motion.forward_motion()
            time.sleep(0.01)

    def back_motion(self):
        while not self.stop_motion:
            motion.back_motion()
            time.sleep(0.01)

    def left_motion(self):
        while not self.stop_motion:
            motion.left_motion()
            time.sleep(0.01)

    def right_motion(self):
        while not self.stop_motion:
            motion.right_motion()
            time.sleep(0.01)

    def talk_motion(self):
        speak_and_listen.talk()
        time.sleep(0.01)

def main (args=None):
    rclpy.init(args=args)
    command_subscriber_action = CommandSubscriberAction()
    try:
        rclpy.spin(command_subscriber_action)
    except KeyboardInterrupt:
        print("Shutting down by Ctrl+C...")
    finally:
        command_subscriber_action.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("プログラム終了")

if __name__ == '__main__':
    main()
