while getopts sr flag
do
    case "${flag}" in
        s) 
        roslaunch ~/Robotics_Final_Project/catkin_ws/launch/sim.launch
        echo "here"
        ;;
        r)
        roslaunch ~/Robotics_Final_Project/catkin_ws/launch/real.launch
        echo "here2"
        ;;
    esac
done