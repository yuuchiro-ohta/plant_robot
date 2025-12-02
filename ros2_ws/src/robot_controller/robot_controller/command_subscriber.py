import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import robot_controller.motion as motion
import robot_controller.speak_and_listen as speak_and_listen
import time
import threading

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
       
    def listener_callback(self, msg):
        cmd = msg.data.lower()
        self.get_logger().info(f"Received command: {cmd}")

        # 前の動作を止める
        if self.current_thread and self.current_thread.is_alive():
            self.get_logger().warn("Stopping previous motion...")
            self.stop_motion = True
            self.current_thread.join()

        self.stop_motion = False
           
        if cmd == "forward":
            self.start_motion(self.forward_motion)
        elif cmd == "back":
            self.start_motion(self.back_motion)
        elif cmd == "left":
            self.start_motion(self.left_motion)
        elif cmd == "right":
            self.start_motion(self.right_motion)
        elif cmd == "talk":
            self.start_motion(self.talk_motion)
        elif cmd == "stop":
            self.start_motion(self.stop)
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    def start_motion(self, motion_func):
        self.current_thread = threading.Thread(target=motion_func)
        self.current_thread.start()
    
    def forward_motion(self):
        motion.forward_motion()
        time.sleep(0.01)

    def back_motion(self):
        motion.back_motion()
        time.sleep(0.01)

    def left_motion(self):
        motion.left_motion()
        time.sleep(0.01)

    def right_motion(self):
        motion.right_motion()
        time.sleep(0.01)

    def talk_motion(self):
        motion.stop()
        speak_and_listen.talk()
        time.sleep(0.01)

    def stop(self):
        self.stop_motion = False
        motion.stop()
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
