
from controller import Robot

def run_robot(robot):
    time_step = 32
    max_speed = 6.28
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    left_ir = robot.getDevice('ir0')
    left_ir.enable(time_step)
    
    right_ir = robot.getDevice('ir1')
    right_ir.enable(time_step)

    while robot.step(time_step) != -1:
    
        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()
        
        print("left: {} right: {}".format(left_ir_value,right_ir_value))
        
        left_speed = max_speed*0.85
        right_speed = max_speed*0.85
        
        if (left_ir_value > right_ir_value) and (0 < left_ir_value < 200):
        
            print("Virei a esquerda")
            left_speed = -max_speed*0.85
            
        elif (right_ir_value > left_ir_value) and  (0 < right_ir_value < 200):
            
            print("Virei a direita")
            right_speed = -max_speed*0.85
            
        elif (right_ir_value > left_ir_value) and  (0 < left_ir_value < 200):
            
            print("Virei a direita")
            right_speed = -max_speed*0.85 
            
        elif (left_ir_value > right_ir_value) and (0 < right_ir_value < 200):
        
            print("Virei a esquerda")
            left_speed = -max_speed*0.85
                  
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)