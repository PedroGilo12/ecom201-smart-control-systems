"""
This code has been developed by Juan Sandubete Lopez and all the rights
belongs to him.
Distribution or commercial use of the code is not allowed without previous
agreement with the author.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import random
import time
import pandas as pd
import random

# Simulation parametrs
tf = 6.0  # final time
ts_ms = 0.01  # 0.001 = 1us, 1 = 1ms
save_data = False  # Attention: CSV file can be very big
show_fig = False
save_fig = True  # If False, figure is showed but not saved

tempo = time.time()

title = f"motor_control_a_with_error_{tempo}"

print("Starting motor simulation.")

# Models Parameters
# Motor
a = 1
k = 1
a_model_error = 0.1
k_model_error = 0.3
# Motor Controller (Control1)
k_c1 = 1

# PID Parameters
Kp = 30
Ki = 0.05
Kd = 0.1

i_erro = 0
last_erro = 0

mseResult = []
mseSum = 0

itaeSum = 0
itaeResult = []

maeSum = 0
maeResult = []

GoodhartE1Sum = 0
GoodhartE1Result = []

GoodhartE2Sum = 0
GoodhartE2Result = []

GoodhartE3Sum = 0
GoodhartE3Result = []

GoodhartResult = []

print("\n--- PARAMETERS --- \n ")
print("Motor Parameters: a = {}, k = {}".format(a, k))
print(
    "Motor Induced Model Errors: a_error = {}, k_error = {}".format(
        a_model_error, k_model_error
    )
)
print("Motor Controller: kc1 = {}".format(k_c1))


# Define models
def dc_motor_model(x1_m, u):
    # DC motor model:
    # taup + a*k*tau = k*u
    # With the change: x1_m = tau (x1_motor)
    dx1_m = -a * k * x1_m + k * u
    y1_m = x1_m
    return dx1_m


def PID_controller(tau, tau_ref, taup_ref):
    global i_erro
    global last_erro
    erro = tau_ref - tau

    i_erro = i_erro + (erro * ts_ms)
    d_erro = (erro - last_erro) / ts_ms

    Up = erro * Kp
    Ui = i_erro * Ki
    Ud = d_erro * Kd

    last_erro = erro

    return Up + Ui + Ud

def motor_controller(tau, tau_ref, taup_ref):
    # Non-Linear control for DC Motor following Dyn ecs: taup + a*k*tau = k*u
    # The controller returns dc_volts
    v = taup_ref - k_c1 * (tau - tau_ref)
    return (a + a_model_error) * tau + v / (k + k_model_error)

def MSE(erro, time, n):
    global mseSum
    global mseResult

    mseSum = mseSum + np.power(erro, 2)
    mseResult.append(mseSum/n)

def ITAE(erro, time, n):
    global itaeSum
    global itaeResult
    itaeSum = itaeSum + (time * abs(erro))
    itaeResult.append((1 / n) * itaeSum)
    return itaeResult[-1]


def MAE(erro, time, n):
    global maeSum
    global maeResult
    maeSum = maeSum + abs(erro)
    maeResult.append((1 / n) * maeSum)
    return maeResult[-1]


def GoodhartE1(x1_m, n):
    global GoodhartE1Sum
    global GoodhartE1Result
    GoodhartE1Sum = GoodhartE1Sum + x1_m
    GoodhartE1Result.append((1 / n) * GoodhartE1Sum)
    return GoodhartE1Result[-1]


def GoodhartE2(x1_m, n):
    global GoodhartE2Sum
    global GoodhartE2Result
    GoodhartE2Sum = GoodhartE2Sum + ((x1_m - GoodhartE1(x1_m, n)) ** 2)
    GoodhartE2Result.append((1 / n) * GoodhartE2Sum)
    return GoodhartE2Result[-1]


def GoodhartE3(erro, n):
    global GoodhartE3Sum
    global GoodhartE3Result
    GoodhartE3Sum = GoodhartE3Sum + (erro**2)
    GoodhartE3Result.append((1 / n) * GoodhartE3Sum)
    return GoodhartE3Result[-1]


def Goodhart(x1_m, erro, n):
    global GoodhartResult
    valor = (
        ( GoodhartE1(x1_m, n) * 0.33)
        + (GoodhartE2(x1_m, n) * 0.33)
        + (GoodhartE3(erro, n) * 0.34)
    )
    GoodhartResult.append(valor)
    return GoodhartResult[-1]


def PID_connected_systems_model(states, t, tau_ref, taup_ref):
    # Input values. Check this with the out_states list
    x1_m, _ = states

    # Compute motor controller
    dc_volts = PID_controller(x1_m, tau_ref, taup_ref)
    # Compute motor torque
    taup = dc_motor_model(x1_m, dc_volts)

    # Output
    out_states = [taup, dc_volts]
    return out_states

# The following function puts all ecuations together
def connected_systems_model(states, t, tau_ref, taup_ref):
    # Input values. Check this with the out_states list
    x1_m, _ = states

    # Compute motor controller
    dc_volts = motor_controller(x1_m, tau_ref, taup_ref)
    # Compute motor torque
    taup = dc_motor_model(x1_m, dc_volts)

    # Output
    out_states = [taup, dc_volts]
    return out_states


# Initial conditions
states0 = [0, 0]
n = int((1 / (ts_ms / 1000.0)) * tf + 1)  # number of time points

# time span for the simulation, cycle every tf/n seconds
time_vector = np.linspace(0, tf, n)
t_sim_step = time_vector[1] - time_vector[0]

# Reference signal and its differentiations
torque_ref = np.full(time_vector.shape, 5)
print("Max ref: {}".format(max(torque_ref)))
print("Min ref: {}".format(min(torque_ref)))
# torquep_ref = np.full(time_vector.shape, 2) + np.cos(time_vector)
# torquep_ref = np.cos(time_vector)
torquep_ref = np.full(time_vector.shape, 0)
print("Max ref: {}".format(max(torquep_ref)))
print("Min ref: {}".format(min(torquep_ref)))
# Output arrays
states = np.zeros((n - 1, len(states0)))  # States for each timestep

print("\n--- SIMULATION CONFIG. ---\n")
print("Simulation time: {} sec".format(tf))
print("Time granulatiry: {}".format(t_sim_step))
print("Initial states: {}".format(states0))

print("\n--- SIMULATION Begins ---\n")

erroVector = []

initial_time = time.time()
# Simulate with ODEINT
t_counter = 0
for i in range(n - 1):
    out_states = odeint(
        PID_connected_systems_model,
        states0,
        [0.0, tf / n],
        args=(torque_ref[i], torquep_ref[i]),
    )

    x1_m = states0[0]
    u = states0[0]
    erro = torque_ref[-1] - x1_m

    erroVector.append(erro)

    ITAE(erro, i * t_sim_step, i + 1)
    MAE(erro, i * t_sim_step, i + 1)
    MSE(erro, i * t_sim_step, i + 1)
    Goodhart(u, erro, i + 1)

    states0 = out_states[-1, :]
    states[i] = out_states[-1, :]
    if i >= t_counter * int((n - 1) / 10):
        print("Simulation at {}%".format(t_counter * 10))
        t_counter += 1

elapsed_time = time.time() - initial_time
print("\nElapsed time: {} sec.".format(elapsed_time))
print("\n--- SIMULATION Finished. ---\n")

if save_data:
    print("Saving simulation data...")
    sim_df = pd.DataFrame(states)
    sim_df = sim_df.transpose()
    sim_df.rename({0: "tau", 1: "tau_ref", 2: "taup_ref", 3: "dc_volts"}, inplace=True)
    sim_df.to_csv("sim_data/ex4_motor_control.csv")


# Plot results
# States are: tau, tau_ref, taup_ref, dc_volts
print(mseResult[-1])
print(itaeResult[-1])
print(maeResult[-1])
print(GoodhartResult[-1])

plt.rcParams["axes.grid"] = True

plt.figure()
plt.title(f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
plt.subplot(3, 1, 1)
plt.plot(time_vector[:-1], torque_ref[:-1], "k--", linewidth=3)
plt.plot(time_vector[:-1], states[:, 0], "r", linewidth=2)
plt.ylabel("tau [Nm]")
plt.subplot(3, 1, 2)
plt.plot(time_vector[:-1], states[:,1], "g", linewidth=2)
plt.ylabel("Controller output")
plt.subplot(3, 1, 3)
plt.plot(time_vector[:-1], erroVector, "g", linewidth=2)
plt.ylabel("Erro")
plt.xlabel(f"mse: {mseResult[-1]:.6}, ita: {itaeResult[-1]:.6}, mae:{maeResult[-1]:.6} goodhart: {GoodhartResult[-1]}")

if save_fig:
    figname = "pictures/" + title + ".png"
    plt.savefig(figname)
if show_fig:
    plt.show()