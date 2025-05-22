import sys
import os
sys.path.append("..")
import utilitiess.templates as templates
import utilitiess.parse as parser

current_path = os.path.dirname(os.path.realpath(__file__))
print("CURRENT PATH", current_path)
new_path = os.path.join(current_path, '..', '..')
print("NEW PATH", new_path)

class Views:
    def __init__(self, node_info):
        file_path = os.path.join(new_path, 'AUC-Thesis-Leithy-DT/RemoteDrivingDashboard-master/apps/actuate/views.py')

        self.node_info = node_info
        # open views.py file in the file_path and append
        with open(file_path, "r") as file:
            lines = file.readlines()

        new_lines = []
        appended = False

        for line in lines:
            if "#generated code region" in line and node_info.flow_flag == 1:
                index = line.index("#generated code region")
                new_lines.append(line[:index + len("#generated code region")] + "\n\n")
                new_lines.append(f"""
@sio.on('{self.node_info.node_name}_data', namespace='/dashboard')
#Add the definition of your functions here as follows use the fn you created in Middleware
def newData(sid, data):
    print(data)
    sio.emit('fn', data, namespace='/{self.node_info.node_name}_namespace')\n\n""")
                new_lines.append(line[index + len("#generated code region"):])
                appended = True
            else:
                new_lines.append(line)

        if not appended:
            return

        try:
            with open(file_path, 'w') as file:
                file.writelines(new_lines)
        except IOError:
            return

        print("File processed successfully.")

