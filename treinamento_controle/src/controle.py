#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import definitions
import matplotlib.pyplot as plt
import numpy as np

'''PID controller for a robot on Gazebo simulation'''

#Function to get the reference velocity
def get_reference_velocity():
    vel_ref = float(input("Enter the reference velocity: "))
    return vel_ref


#Main function, reponsible for running the control loop
def control_main():
    rospy.init_node('treinamento_controle', anonymous=True) #Se nao funcionar, tentar treinamento_controle
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    #Velocity message and reference velocity
    velocity_msg = Twist()
    velocity_ref = get_reference_velocity()
    current_velocity = definitions.INITIAL_VEL
    velocity_msg.linear.y = 0.0
    velocity_msg.linear.z = 0.0
    velocity_msg.angular.x, velocity_msg.angular.y, velocity_msg.angular.z = 0, 0, 0

    #Initial value for the robot velocity
    velocity_msg.linear.x = current_velocity

    #Integral error (cumulative error)
    integral_error = 0.0
    previous_error = 0.0

    y = [0]
    total_time = 0

    #Initial time
    current_time = past_time = rospy.Time.now()

    while not rospy.is_shutdown():
        while ((current_time - past_time).to_sec() < definitions.PERIOD):
            #Calculate the error
            error = velocity_ref - current_velocity
            integral_error += error

            #Here we limit the integral error
            if integral_error > definitions.MAX_INTEGRAL:
                integral_error = definitions.MAX_INTEGRAL
            elif integral_error < definitions.MIN_INTEGRAL:
                integral_error = definitions.MIN_INTEGRAL

            print(f'Error: {error}')

            #Calculate the pid control value by location formula
            #u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki */
            control_signal = (error*definitions.KP) + ((error - previous_error)*definitions.KD) + (integral_error*definitions.KI)

            print(f'Control signal1: {control_signal}')

            #Here we limit the control signal
            if control_signal > definitions.MAX_VEL:
                control_signal = definitions.MAX_VEL
            elif control_signal < definitions.MIN_VEL:
                control_signal = definitions.MIN_VEL

            print(f'Control signal2: {control_signal}')

            #Here we calculate the new velocity
            current_velocity += control_signal #Is this right?
            y.append(current_velocity)

            print("Current velocity: ", current_velocity)
            #Here we publish the new velocity
            velocity_msg.linear.x = current_velocity
            velocity_publisher.publish(velocity_msg)
            
            current_time = rospy.Time.now()
            previous_error = error

            print(f"Time{(current_time - past_time).to_sec()}")
            print('')
            rospy.sleep(0.05)
        
        #Stop the robot and get another reference velocity
        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)

        #Plotando o gráfico de conversão:
        total_time += definitions.PERIOD
        x = np.linspace(0, total_time, len(y))
        ref = velocity_ref*np.ones_like(x)
        plt.plot(x, y, label='Velocidade Linear (V)')
        plt.plot(x, ref, label='Referência')
        plt.xlabel('Tempo (seg)')
        plt.ylabel('Velocidade Linear (V)')
        plt.title('Resposta do Controlador PID')
        plt.legend()
        plt.grid(True)

        plt.show()

        velocity_ref = get_reference_velocity()
        current_time = past_time = rospy.Time.now()


if __name__ == '__main__':
    try:
        control_main()
    except rospy.ROSInterruptException: pass