from scipy.optimize import minimize
import numpy as np
import pid_auto_tunning
import numpy as np
import random

a = 1
k = 1

def dc_motor_model(x1_m, t, u):
    dx1_m = -a*k*x1_m + k*u
    return dx1_m

def generate_random_pid_gains(n=10):
    pid_list = []
    for _ in range(n):
        Kp = round(random.uniform(0, 50), 3)
        Ki = round(random.uniform(0, 10), 3)
        Kd = round(random.uniform(0, 5) , 3)
        pid_list.append([Kp, Ki, Kd])
    return pid_list

initial_set = generate_random_pid_gains(10)

auto_tunning = pid_auto_tunning.PIDAutoTunning(dc_motor_model)
parcial_result = auto_tunning.tunning(initial_set, False)
parcial_result = sorted(parcial_result, key=lambda x: x[1])

pid_gains = parcial_result[0][0]

auto_tunning = pid_auto_tunning.PIDAutoTunning(dc_motor_model)
auto_tunning.set_save_figs(True)

result = minimize(
    auto_tunning.run_pid,
    pid_gains,
    method='Nelder-Mead',
    options={
        "maxiter": 100,      
        "fatol": 1e-2,       
        "xatol": 1e-2        
    }
)

print("Optimization successful:", result.success)
print("Message:", result.message)
print("Optimal parameters (x):", result.x)
print("Minimum function value (fun):", result.fun)
print("Number of function evaluations (nfev):", result.nfev)