#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import definitions

'''PID controller for a robot on Gazebo simulation'''

#Defining the function to get the reference velocity
def get_reference_velocity():
    vel_ref = float(input("Enter the reference velocity: "))
    return vel_ref


#Defininf the main function, reponsible for running the control loop
def control_main():
    rospy.init_node('Projeto-controle-Semear', anonymous=True) #Se nao funcionar, tentar treinamento_controle
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    #Defining the velocity message and reference velocity
    velocity_msg = Twist()
    velocity_ref = get_reference_velocity()
    current_velocity = definitions.INITIAL_VEL
    velocity_msg.linear.y = 0.0
    velocity_msg.linear.z = 0.0
    velocity_msg.angular.x, velocity_msg.angular.y, velocity_msg.angular.z = 0, 0, 0

    #Defining the initial value for the robot velocity
    velocity_msg.linear.x = current_velocity

    while not rospy.is_shutdown():
        #Calculate the error
        error = velocity_ref - current_velocity

        #Calculate the control signal using a PID controller
        #Stil dont know how to do it
        #Consider the period during the pid execution? thinking about doing a while loop that runs for the PID during that period time
        control_signal = 0.2

        #Here we limit the control signal
        if control_signal > definitions.MAX_VEL:
            control_signal = definitions.MAX_VEL
        elif control_signal < definitions.MIN_VEL:
            control_signal = definitions.MIN_VEL

        #Here we calculate the new velocity
        current_velocity += current_velocity + control_signal*definitions.PERIOD #Is this right?

        #Here we publish the new velocity
        velocity_msg.linear.x = current_velocity
        velocity_publisher.publish(velocity_msg)

        velocity_ref = get_reference_velocity
if __name__ == '__main__':
    try:
        control_main()
    except rospy.ROSInterruptException: pass