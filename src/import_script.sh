address=montaulab@192.168.1.28
serveur_path=/home/montaulab/workspaces/isaac_ros-dev
port=22
scp -P $port $address:$serveur_path/src/course_v1/course_v1/*.py .
scp -P $port $address:$serveur_path/src/course_v1/course_v1/templates/*.html .
scp -P $port $address:$serveur_path/launch/*.py .
scp -P $port $address:/home/montaulab/esp32/*/*.ino .
