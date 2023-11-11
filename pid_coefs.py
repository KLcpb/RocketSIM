import matplotlib.pyplot as plt
import sys
import math

# M = I*e
# A = kp * e + ki * intergal(e dt) + kd * de/dt
# dF(a,b) = 
# тут оказывается не нужно чтобы моменты были равны(

# ---------------mechanical params
dt = 0.001
mass = 1
I = mass*0.30225
sim_time = 2.5
engine_time = 2.5
g = 9.81
AB = 0.15 #distance to COM
# ----------------koef
kp = 0.003
ki = 0
kd = 0.025

err_log = []

def get_aerodynamic_moment(alpha,v): # подберем
    # pass
    return -math.sin(alpha)*v*0.1

def get_control_moment(beta,thrust):
    return thrust * AB * math.sin(beta)

def thrust(t):  # подберем
    a = -300*(t-0.8)**2 + 121
    if a>0:
        return a
    else:
        return 0

n = 0 #intergal for pid controller

def pid(err,prev_err,kp,ki,kd):
    global n
    n = n + err*dt
    beta = kp * err + ki * n + kd * (err - prev_err)/dt
    return beta


def sim(kp,ki,kd):
    global n
    n = 0
    v = 0
    eps = 0
    err = 0.1 #устанавливая начальную ошибку 
    # добьемя адекватного вида графика ошибкu
    preverr = 0
    beta = 0
    W = 0
    accZ = 0
    err_log = []
    global sim_time,dt,mass,g,I
    for i in range(int(sim_time / dt)):#quantity of iterations
        accZ = thrust(i*dt) / mass - g 
        v = v + accZ * dt

        eps =  (get_aerodynamic_moment(err,v) - get_control_moment(beta,thrust(i*dt)))/I

        W = W + eps*dt
        err = err + W * dt #вычилсяем угол отклонения интегрированием
        beta = pid(err,preverr,kp,ki,kd)
        preverr = err

        err_log.append(err)
        # print(err,eps,v,beta)
        if err > 0.5:
            return err_log
    return err_log
plt.plot(sim(0.1,0,0.1))
# plt.plot(sim(0.003,0,0.025))
# plt.plot(sim(0.01,0,0.05))
plt.show()

