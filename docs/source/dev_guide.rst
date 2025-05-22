.. _developer_guide:

===============
Developer Guide
===============

This guide is intended to show developers how to use the SDK to create their own digital twin applications

Generating a Digital Twin
=========================

YAML File
---------
The YAML file will include all the ros nodes that will be in the digital twin and from this file all worklow is going to be created. 
Files created from the yaml file will create a files in each layer to relay from the ROS layer to the middleware layer and to the namespace layer.

The yaml file is found under the SDK_NL directory inside the ros_layer directory.

The structure of the YAML file is as follows:

.. code-block:: yaml

    ros_node:
        name: node_name
        flow_flag: 0         #(1:Bottom-Up Flow, 0:Top-Down Flow)
        topics:
            publish:
                topic_1:
                    name: "middleware/test"
                    type:  "nav_msgs/Odometry"
                    middleware_flag: 1    
            subscribe:
                topic_1:
                    name: "test"
                    type:  "nav_msgs/Odometry"
                    middleware_flag: 1

**YAML File Structure:**

-  **ros_node**: This is the main key for the YAML file. It will include details about the node needed.
-  **name**: This is the name of the ROS node.
-  **flow_flag**: This flag is used to determine the flow of the data. If the flag is set to 1, the data will flow from the namespace layer to the middleware layer to the ROS layer. If the flag is set to 0, the data will flow from the ROS layer to the middleware layer to the namespace layer.
-  **topics**: This key will include the topics that the node will publish and subscribe to.
-  **publish**: This key will include the topics that the node will publish to.
-  **subscribe**: This key will include the topics that the node will subscribe to.
-  **topic_1**: This key will include the details of the topic.
-  **name**: This is the name of the topic.
-  **type**: This is the message type of the topic.

**Note**: when inputting the name of the topic, the name shouldn't include / at the beginning. For now the node_config.yaml file exists in the SDK_NL/ros_layer directory.



Code Generator
--------------

After creating the YAML file, the following steps generate files for the ROS layer and the middleware layer, the following steps are needed:

.. code-block:: bash

    $ cd SDK_NL/ros_layer
    $ python3 node_generator.py

This will generate code for the ROS layer and the middleware layer. The generated files will be named in {node_name}_ros.py and {node_name}_middleware.py. 

-  **ROS Layer**: AUC-Thesis-Leithy-DT/ROS-master
-  **Middleware Layer**: AUC-Thesis-Leithy-DT/ROS-master/cloudconnect
-  **Namespace Layer**: AUC-Thesis-Leithy-DT/RemoteDrivingDashboard-master/apps/actuate/views.py


Namespace Layer
~~~~~~~~~~~~~~~

The namesapce layer files are already generated only the following code snippet from the views.py file is generated when the node_generator.py is run:

.. code-block:: python

    #Generated Code Region

    @sio.on('latency_test_data', namespace='/dashboard')
    #Add the definition of your functions here as follows use the fn you created in Middleware
    def newData(sid, data):
        print(data)
        sio.emit('fn', data, namespace='/latency_test_namespace')

    #End of Generated Region

ROS Layer and Middleware Layer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once you locate the generated files, you can start editing the files to add the functionality you need in the callback functions.

Bidirectional Communication
---------------------------
For the bidirectional communication, you should generate a your code from the YAML file once with the *flow_flag* set to 0 and another time with the *flow_flag* set to 1. Make sure that the two nodes are not under the same master node.

Running the Digital Twin
========================

Once you have generated all the files and edited the code, you can run the digital twin by running each layer step-by-step.

Dockerized Private Cloud
------------------------
To run the Dockerized Private Cloud and its services, you need to run the following:

.. code-block:: bash

    $ cd AUC-Thesis-Leithy-DT/AUC-Thesis-DT-Physical/RemoteDrivingDashboard-master
    $ export $HOST_IP=localhost
    $ docker-compose up

Now you will have the private dockerized cloud aka the namespace layer running.

Dashboard
---------
The dashboard visualizes the digital twin connections and data. To run the dashboard, you need to run the following:

1. The following file shows the services such as the camera stream or sensory data that you want to visualize.

.. code-block:: bash

    $ cd /AUC-Thesis-Leithy-DT/OTA_RemoteDrivingConfigurator-main/Designs/
    $ python3 QTGUI.py

2. Open the dashboard by going to your browser and go to the **http://localhost:8000** URL.

The dashboard has the necessary functionalities. If your application has more and they are not supported by the dashboard, you can add them by editing the javascript files.

ROS Layer
---------
To run the ROS layer for a specific node, you need to simply run it by going to the directory and running it as follows:
Chnage the *node_name* to the name of the node you want to run.

.. code-block:: bash

    $ cd AUC-Thesis-Leithy-DT/ROS-master
    $ python3 {node_name}_ros.py

Middleware Layer
----------------
To run the middleware layer for a specific node, you need to simply run it by going to the directory and running it as follows:
Chnage the *node_name* to the name of the node you want to run.

.. code-block:: bash

    $ cd AUC-Thesis-Leithy-DT/ROS-master/cloudconnect
    $ python3 {node_name}_middleware.py
