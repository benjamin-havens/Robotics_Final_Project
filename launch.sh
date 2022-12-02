while getopts sr flag
do
    case "${flag}" in
        s) 
        roslaunch ~/Robotics_Final_Project/catkin_ws/launch/sim.launch
        ;;
        r)
        roslaunch ~/Robotics_Final_Project/catkin_ws/launch/real.launch
        ;;
    esac
done