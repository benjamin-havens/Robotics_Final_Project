from Ax12 import Ax12

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = '/dev/ttyUSB0'

Ax12.BAUDRATE = 1000000

# sets baudrate and opens com port
Ax12.connect()

# creates 3 AX12 instance with ID 1,2,3
dxl_1 = Ax12(1)
dxl_2 = Ax12(2)
dxl_3 = Ax12(3)

# Sets moving speed of each motor
dxl_1.set_moving_speed(100)
dxl_2.set_moving_speed(100)
dxl_3.set_moving_speed(100)


def user_input():
    """Check to see if user wants to continue"""
    ans = input('Continue? : y/n ')
    if ans == 'n':
        return False
    else:
        return True


def main(motor_object1, motor_object2, motor_object3):
    """ sets goal position based on user input """

    # 90 degrees is ~ 250
    # -90 degrees is ~ 850

    # Set initial position to -90 degrees
    input_pos2 = 849
    
    motor_object1.set_goal_position(input_pos2)
    motor_object2.set_goal_position(input_pos2)
    motor_object3.set_goal_position(input_pos2)

    while motor_object2.is_moving():
        pass

    while 1:

        print("\nPosition of dxl ID: %d is %d " %
              (motor_object1.id, motor_object1.get_present_position()))
        print("\nPosition of dxl ID: %d is %d " % (motor_object2.id, motor_object2.get_present_position()))
        print("\nPosition of dxl ID: %d is %d " % (motor_object3.id, motor_object3.get_present_position()))
        # desired angle input
        # input_pos = int(input("goal pos1: "))

        input_pos = 300


        # if input_pos < 0:
        #     break
        # input_pos2 = int(input("goal pos2: "))
        # input_pos3 = int(input("goal pos3: "))

        motor_object1.set_goal_position(input_pos)

        while motor_object1.get_present_position() >= 550:
            pass

        motor_object2.set_goal_position(input_pos)

        while motor_object2.get_present_position() >= 550:
            pass

        motor_object3.set_goal_position(input_pos)



        # not_there = True
        # while not_there:
        #     if motor_object1.get_present_position() == input_pos:22
        #         not_there = False

        while motor_object3.is_moving():
            pass

        input_pos2 = 649
        
        motor_object1.set_goal_position(input_pos2)
        while motor_object1.get_present_position() <= 375:
            pass

        motor_object2.set_goal_position(input_pos2)
        while motor_object2.get_present_position() <= 375:
            pass
        motor_object3.set_goal_position(input_pos2)

        while motor_object3.is_moving():
            pass
        # not_there = True

        # while not_there:
        #     if motor_object1.get_present_position() == input_pos2:
        #         not_there = False

        print("Position of dxl ID: %d is now: %d " %
              (motor_object1.id, motor_object1.get_present_position()))
        print("Position of dxl ID: %d is now: %d " % 
                (motor_object2.id, motor_object2.get_present_position()))
        print("Position of dxl ID: %d is now: %d " %
                (motor_object3.id, motor_object3.get_present_position()))
        # bool_test = user_input()

# pass in AX12 objects
main(dxl_1, dxl_2, dxl_3)

# disconnect
dxl_1.set_torque_enable(0)
dxl_2.set_torque_enable(0)
dxl_3.set_torque_enable(0)
Ax12.disconnect()