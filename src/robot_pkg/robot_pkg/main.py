import rclpy
import rclpy.logging

def main():
    rclpy.logging.get_logger("Test").info("Hello!")