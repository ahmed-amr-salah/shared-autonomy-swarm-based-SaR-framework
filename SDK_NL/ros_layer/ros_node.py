import rospy
from geometry_msgs.msg import Twist
import sys, select, os
from std_msgs.msg import String
import sys

sys.path.append("..")
print("PATH: ", sys.path)
import utilitiess.templates as templates
import utilitiess.parse as parser

current_path = os.path.dirname(os.path.realpath(__file__))
print("CURRENT PATH", current_path)
new_path = os.path.join(current_path, '..', '..')
print("NEW PATH", new_path)

class ROSNodeInformation:
    def __init__(self, node_name, node_subscribers, node_publishers, flow_flag):
        self.node_name = node_name
        self.flow_flag = flow_flag
        self.node_subscribers = node_subscribers
        self.node_publishers = node_publishers


class Node:
    def __init__(self, ros_node_info):  # Constructor
        file_path = os.path.join(new_path, 'AUC-Thesis-Leithy-DT/ROS-master', f"{ros_node_info.node_name}_ros.py")

        with open(file_path, "w") as file:
            pass  # Using pass statement since we don't need to write anything
        self.nodeInfo = ros_node_info
        with open(file_path, "a") as file:
            # parse imports types
            writer = templates.FileWriterr(file=file)
            imports, types = parser.Parser.parse_types(ros_node_info)
            # funciton to write the imports in the file
            writer.write_imports(imports, types)

            print(ros_node_info.node_subscribers)
            writer.callback_function(ros_node_info.node_subscribers, ros_node_info.node_publishers) 

            writer.init_node(ros_node_info.node_name)

            writer.write_publishers(ros_node_info.node_publishers)
            writer.write_subscribers(ros_node_info.node_subscribers)
            file.write(f"""if __name__ == '__main__':\n""")  
            file.write(f"""    rospy.spin()""")
        parse = parser.Parser.parse_types(ros_node_info)
