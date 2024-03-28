
from controller import Robot

robot = Robot()
time_step = 2

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

sensors = []
names = ["ir0","ir1","ir2","ir3","ir4","ir5","ir6","ir7"]
reading = [0,0,0,0,0,0,0,0]

for i in range (0,8):
    sensors.append(robot.getDevice(names[i]))
    sensors[i].enable(time_step)

def getReading():
    for i in range (0,8):
        if (sensors[i].getValue())>400: #está na linha
            reading[i] = 0
        else:
            reading[i] = 1
            
        print(reading)
            
    return reading
        
def erro_calculator(reading):

    erro = 0

    if ((reading[0]==1) and (reading[1]==1) and (reading[2]==1) and (reading[3]==1) and (reading[4]==1) and (reading[5]==1) and (reading[6]==1) and (reading[7]==1)):
        erro=0 #todos os sensores estão na linha
    elif ((reading[0]==1) and (reading[1]==1) and (reading[2]==1) and (reading[3]==1) and (reading[4]==0) and (reading[5]==0) and (reading[6]==0) and (reading[7]==0)):
        erro=1 #deve virar a direita
    elif ((reading[0]==1) and (reading[1]==1) and (reading[2]==1) and (reading[3]==1) and (reading[4]==1) and (reading[5]==0) and (reading[6]==0) and (reading[7]==0)):
        erro=1 #deve virar a direita
    elif ((reading[0]==0) and (reading[1]==0) and (reading[2]==0) and (reading[3]==0) and (reading[4]==1) and (reading[5]==1) and (reading[6]==1) and (reading[7]==1)):
        erro=-1 #deve virar a esquerda
    elif ((reading[0]==1) and (reading[1]==1) and (reading[2]==1) and (reading[3]==0) and (reading[4]==0) and (reading[5]==0) and (reading[6]==0) and (reading[7]==0)):
        erro=1.5 #deve virar a direita
    elif ((reading[0]==0) and (reading[1]==0) and (reading[2]==0) and (reading[3]==0) and (reading[4]==0) and (reading[5]==1) and (reading[6]==1) and (reading[7]==1)):
        erro=-1.5 #deve virar a esquerda
    elif ((reading[0]==1) and (reading[1]==1) and (reading[2]==0) and (reading[3]==0) and (reading[4]==0) and (reading[5]==0) and (reading[6]==0) and (reading[7]==0)):
        erro=1.75 #deve virar a direita
    elif ((reading[0]==0) and (reading[1]==0) and (reading[2]==0) and (reading[3]==0) and (reading[4]==0) and (reading[5]==0) and (reading[6]==1) and (reading[7]==1)):
        erro=-1.75 #deve virar a esqeurda
    elif ((reading[0]==1) and (reading[1]==0) and (reading[2]==0) and (reading[3]==0) and (reading[4]==0) and (reading[5]==0) and (reading[6]==0) and (reading[7]==0)):
        erro=2 #deve virar a direita
    elif ((reading[0]==0) and (reading[1]==0) and (reading[2]==0) and (reading[3]==0) and (reading[4]==0) and (reading[5]==0) and (reading[6]==0) and (reading[7]==1)):
        erro=-2 #deve virar a esquerda
    elif ((reading[0]==0) and (reading[1]==0) and (reading[2]==1) and (reading[3]==1) and (reading[4]==1) and (reading[5]==1) and (reading[6]==0) and (reading[7]==0)):
        erro=0 #deve andar em linha reta
    return erro

def pid_calculator(erro):
    
    global erro_anterior
    
    kp = 35
    ki = 0
    kd = 35
    erro_anterior = 0
    
    if erro == 0:
        i = 0
    p = erro
    i = ki + erro
    d = erro - erro_anterior
    pid = (kp*p) + (ki*i) + (kd*d)
    erro_anterior = erro
    
    print(pid)
    
    return pid
    
def run_robot(max_speed):   
    while(robot.step(time_step) != -1):
    
        reading = getReading()
        erro = erro_calculator(reading)
        pid = pid_calculator(erro)
        
        if pid == 0:
            left_speed = max_speed
            right_speed = max_speed
        elif pid < 0:
            left_speed = max_speed + pid
            right_speed = max_speed
        elif pid > 0:
            left_speed = max_speed
            right_speed = max_speed - pid
            
        if left_speed < 0: left_speed = 0
        elif left_speed > 10: left_speed = 10
        
        if right_speed < 0: right_speed = 0
        elif right_speed > 10: right_speed = 10
                
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
if __name__ == "__main__":
    max_speed = 6.28
    run_robot(max_speed)