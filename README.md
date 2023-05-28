# gladius_ros
Gladius robot examples using Robot Operating System.

# Permisions to Arduino port
sudo chmod a+rw /dev/ttyUSB0

# Build image from Dockerfile
docker build -t gladius_ros .

# Run Docker containers
docker run -it --net=host gladius_ros roscore

docker run -it --privileged --net=host -v /dev/ttyUSB0:/dev/ttyUSB0 gladius_ros

# Run rosserial in second container
rosrun rosserial_python serial_node.py /dev/ttyUSB0