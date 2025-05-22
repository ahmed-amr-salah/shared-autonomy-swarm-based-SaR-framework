import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import time
import math
import tkinter as tk
from threading import Thread
import sys
import csv
import os


class Actuate(Node):
    def __init__(self, robot_max=3):
        super().__init__('actuate_node')

        self.robot_max = robot_max
        self.robot_num = 0

        self.goal_area_center = (8.437060, 0.321622)
        self.goal_area_radius = 3.0

        self.turtlebot_pubs = []
        self.boolean_pubs = []
        self.autonomous = []

        for i in range(self.robot_max):
            self.turtlebot_pubs.append(self.create_publisher(Twist, f'/robot_{i}/HumanControl', 10))
            self.boolean_pubs.append(self.create_publisher(Bool, f'/robot_{i}/Mode', 10))
            self.autonomous.append(True)
            self.create_subscription(Odometry, f'/robot_{i}/odom', self.make_pose_callback(i), 10)

        self.create_subscription(String, '/control', self.key_callback, 10)

        self.goal_reached = [False] * self.robot_max
        self.start_times = [time.time()] * self.robot_max
        self.coord_labels = {}

        self.send_initial_zero_velocity()

        self.human_interventions = 0
        self.human_control_time = [0.0] * self.robot_max
        self.autonomous_time = [0.0] * self.robot_max
        self.last_mode_switch_time = [time.time()] * self.robot_max

        self.live_window_thread = Thread(target=self.create_live_window, daemon=True)
        self.live_window_thread.start()

        self.show_current_goal_popup()

        # Start timeout watchers
        for i in range(self.robot_max):
            Thread(target=self.watch_timeout, args=(i,), daemon=True).start()

        # Prepare CSV
        self.csv_file = "results.csv"
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Robot ID", "Status", "Total Time (s)", "Interventions", "Human Time (s)", "Auto Time (s)"])
    
    def gui_available(self):
        return os.environ.get('DISPLAY') is not None

    def send_initial_zero_velocity(self):
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.turtlebot_pubs[self.robot_num].publish(zero_twist)

    def key_callback(self, msg):
        data = msg.data
        current_time = time.time()

        if data == 'm':
            if self.autonomous[self.robot_num]:
                self.human_interventions += 1

            self.autonomous_time[self.robot_num] += current_time - self.last_mode_switch_time[self.robot_num]
            self.last_mode_switch_time[self.robot_num] = current_time
            boolean_msg = Bool()
            self.autonomous[self.robot_num] = not self.autonomous[self.robot_num]
            boolean_msg.data = self.autonomous[self.robot_num]
            self.boolean_pubs[self.robot_num].publish(boolean_msg)

        elif data == 'n':
            self.robot_num = (self.robot_num + 1) % self.robot_max

        else:
            twist = Twist()
            if data == 'w':
                twist.linear.x = 0.3
            elif data == 's':
                twist.linear.x = -0.3
            elif data == 'a':
                twist.angular.z = 1.0
            elif data == 'd':
                twist.angular.z = -1.0
            elif data in [' ', 'f']:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.turtlebot_pubs[self.robot_num].publish(twist)

    def make_pose_callback(self, robot_id):
        def callback(msg):
            pos = msg.pose.pose.position
            self.update_live_coords(robot_id, pos.x, pos.y)

            if self.goal_reached[robot_id]:
                return

            dx = pos.x - self.goal_area_center[0]
            dy = pos.y - self.goal_area_center[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < self.goal_area_radius:
                current_time = time.time()
                if self.autonomous[robot_id]:
                    self.autonomous_time[robot_id] += current_time - self.last_mode_switch_time[robot_id]
                else:
                    self.human_control_time[robot_id] += current_time - self.last_mode_switch_time[robot_id]

                total_time = current_time - self.start_times[robot_id]
                self.goal_reached[robot_id] = True

                self.write_result_to_csv(robot_id, "Success", total_time,
                                         self.human_interventions,
                                         self.human_control_time[robot_id],
                                         self.autonomous_time[robot_id])

                Thread(target=self.show_summary_popup, args=(
                    robot_id, total_time,
                    self.human_interventions,
                    self.human_control_time[robot_id],
                    self.autonomous_time[robot_id]
                ), daemon=True).start()

        return callback

    def write_result_to_csv(self, robot_id, status, total_time, interventions, human_time, auto_time):
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([robot_id, status, f"{total_time:.2f}", interventions,
                             f"{human_time:.2f}", f"{auto_time:.2f}"])

    def watch_timeout(self, robot_id):
        while not self.goal_reached[robot_id]:
            time.sleep(5)
            if time.time() - self.start_times[robot_id] > 300:
                current_time = time.time()
                total_time = current_time - self.start_times[robot_id]
                self.goal_reached[robot_id] = False
                self.write_result_to_csv(robot_id, "Failed", total_time,
                                         self.human_interventions,
                                         self.human_control_time[robot_id],
                                         self.autonomous_time[robot_id])
                Thread(target=self.show_popup, args=(f"Robot {robot_id} FAILED the mission (timeout)",), daemon=True).start()
                break

    def show_summary_popup(self, robot_id, total_time, interventions, human_time, auto_time):
        message = (
            f"Robot {robot_id} reached the goal!\n"
            f"Total Time: {total_time:.2f} s\n"
            f"Human Interventions: {interventions}\n"
            f"Human Control Time: {human_time:.2f} s\n"
            f"Autonomous Time: {auto_time:.2f} s\n\n"
            f"🎉 Finished!"
        )
        # if self.gui_available():
        #     self.show_popup(message)
        # else:
        print(message)

    def show_popup(self, message):
        root = tk.Tk()
        root.title("Fleet Autonomy")
        root.geometry("300x200")
        label = tk.Label(root, text=message, font=("Arial", 12), wraplength=250)
        label.pack(pady=20)
        ok_button = tk.Button(root, text="OK", command=root.destroy)
        ok_button.pack()
        root.mainloop()

    def create_live_window(self):
        self.live_root = tk.Tk()
        self.live_root.title("Robot Live Tracker")

        for i in range(self.robot_max):
            self.coord_labels[i] = tk.Label(self.live_root, text=f"Robot {i}: Waiting...", font=("Courier", 14))
            self.coord_labels[i].pack(padx=20, pady=5)

        self.live_root.mainloop()

    def update_live_coords(self, robot_id, x, y):
        coord_text = f"X: {x:.2f}, Y: {y:.2f}"
        if robot_id in self.coord_labels:
            self.coord_labels[robot_id].config(text=f"Robot {robot_id}:\n{coord_text}")

    def show_current_goal_popup(self):
        x, y = self.goal_area_center
        Thread(target=self.show_popup, args=(f"New Goal: Reach the area near ({x}, {y})",), daemon=True).start()

def main(args=None):
    rclpy.init(args=args)

    robot_max = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print("robot max =", robot_max)
    node = Actuate(robot_max)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
