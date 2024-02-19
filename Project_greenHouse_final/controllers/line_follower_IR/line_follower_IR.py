#### Controller for the pesticide spraing robot to go autonomously along the plant rows in the GreenHouse.

from controller import Robot
import math
import matplotlib.pyplot as plt
import numpy as np

def run_robot (robot) :
    time_step = int(robot.getBasicTimeStep())
    max_speed = 3
    wheel_radius = 0.025
    distance_between_wheels = (0.09+0.01)
    # wheel_circum = 2*3.14*wheel_radius
    # encoder_unit = wheel_circum/6.28

    # ## connecting to device sensors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity (0.0)
    
    # enabling wheel sensors
    left_ps = robot.getDevice('left wheel sensor')
    right_ps = robot.getDevice('right wheel sensor')
    
    left_ps.enable(time_step)
    right_ps.enable(time_step)
    
    old_ps=[0, 0]
    new_ps=[0, 0]
    
    x_n_1= -1.848
    y_n_1= -2.37074
    theta_n_1= 1.5708
    
    x_pos_vect = []
    y_pos_vect = []
    
    stopx = -1.848
    
    row_count = 0
    old_spray_flag = False
    
    #PLOT to track the , movement of the robot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    line1, = ax.plot(np.array(x_pos_vect), np.array(y_pos_vect), 'b-',label='Theta = 0')
    ax.legend()
    
    imu = robot.getDevice('imu')
    imu.enable(time_step)
    
    # ##Enabling all 4 ir sensors
    left_ir = robot.getDevice('ds0')
    left_ir.enable(time_step)
    
    mid1_ir = robot.getDevice('ds1')
    mid1_ir.enable(time_step)
    
    mid2_ir = robot.getDevice('ds2')
    mid2_ir.enable(time_step)
    
    right_ir = robot.getDevice('ds3')
    right_ir.enable(time_step)
    
    status_display = robot.getDevice('display')
    
    camera = robot.getDevice('camera')
    camera.enable(time_step)
   
    # ##Step simulation
    while robot.step(time_step) != -1:
        t = robot.getTime()
            
        # ##reading ir sensors
        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()
        mid1_ir_value = mid1_ir.getValue()
        mid2_ir_value = mid2_ir.getValue()
  
        left_speed = max_speed
        right_speed = max_speed
        
        new_ps[0]=left_ps.getValue()*wheel_radius
        new_ps[1]=right_ps.getValue()*wheel_radius
        
        left_wheel_vel = (new_ps[0]-old_ps[0])/(time_step*0.001)
        right_wheel_vel = (new_ps[1]-old_ps[1])/(time_step*0.001)
        
        angles = imu.getRollPitchYaw()
        
        
        #calculating X_dot, Y_dot of the robot coordinates 
        X_dot = (left_wheel_vel + right_wheel_vel)/2.0
        Y_dot = 0
        
        
        #Calculating the velocities for the world's coordinates 
        x_n_dot_i = math.cos(theta_n_1)*X_dot - math.sin(theta_n_1)*Y_dot
        y_n_dot_i = math.sin(theta_n_1)*X_dot + math.cos(theta_n_1)*Y_dot
        
        
        #solve the differential equations to find the robot's position 
        x_n = x_n_dot_i*time_step*0.001 + x_n_1
        y_n = y_n_dot_i*time_step*0.001 + y_n_1
        theta_n = angles[2]
        
        spray_flag = False
        
        if ((y_n-201.05)>-2.224) and ((y_n-201.05)<2) :
            if not old_spray_flag :
                row_count += 1
                print("Spraying Pesticide on Row {} plants".format(row_count))
            spray_flag = True
            
        
        old_spray_flag = spray_flag    
        
        ### commands for the robots to move along the colored path using the IR readings.
        if (left_ir_value > right_ir_value) and (1000<left_ir_value < 1050):
            # print( "Go left" )
            left_speed = -max_speed 
        elif (right_ir_value > left_ir_value) and (1000 < right_ir_value < 1050) :
            # print("Go right")
            right_speed = -max_speed 
        elif (mid1_ir_value > mid2_ir_value) and (1000<mid1_ir_value < 1050):
            # print( "Go mid left" )
            left_speed = -max_speed
            # print("started Spraying")
        elif (mid2_ir_value > mid1_ir_value) and (1000 < mid2_ir_value < 1050) :
            # print("Go mid right")
            right_speed = -max_speed 
        elif ((mid1_ir_value > 600) and (mid1_ir_value<650)) or ((mid2_ir_value > 600) and (mid2_ir_value<650)):
            if ((right_ir_value == mid2_ir_value)and (left_ir_value > right_ir_value)) or (mid1_ir_value > mid2_ir_value):
                # print("right")
                right_speed = -max_speed
            elif ((left_ir_value == mid1_ir_value) and (left_ir_value < right_ir_value)) or (mid2_ir_value > mid1_ir_value):
                left_speed = -max_speed
        if t > 456.224:
            left_speed=0
            right_speed=0   

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        print(str(x_n)+'     '+str(y_n-201.05)+'     '+str(theta_n))
    
        old_ps[0] = new_ps[0]
        old_ps[1] = new_ps[1]
        
        x_n_1 = x_n
        y_n_1 = y_n
        theta_n_1 = theta_n
        
            
        x_pos_vect.append(x_n_1)
        y_pos_vect.append((y_n-201.05))

        line1.set_xdata(np.array(x_pos_vect))
        line1.set_ydata(np.array(y_pos_vect))
        line1.set_label('Theta = %.3f' %theta_n_1)
        ax.legend()
        fig.canvas.draw()
        fig.canvas.flush_events()
        
        
        if left_speed==0 and right_speed==0:
            plt.ioff()
            plt.show()
            break
        
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
    
    




