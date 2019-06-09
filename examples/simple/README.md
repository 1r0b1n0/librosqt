# librosqt simple example
Simple example thats shows how to initialize a node that combines the event loops from ROS and Qt.

This node creates two timers : one `QTimer` and one ROS timer. The callback of both timers will be called from the same thread.

Moreover the node will also subscribe to the topic /chat. The callback of this subscriber will also be called from the same thread as the timers.
