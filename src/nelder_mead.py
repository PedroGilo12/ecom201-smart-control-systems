from scipy.optimize import minimize
import numpy as np
import pid_auto_tunning
import numpy as np
import random
import time

a = 1
k = 1

def dc_motor_model(x1_m, t, u):
    dx1_m = -a*k*x1_m + k*u
    return dx1_m

def thermal_model(T, t, u):
    T_ambient = 25.0
    tau = 5.0
    K = 2.0

    dT = -(T - T_ambient)/tau + K*u
    return dT

def generate_random_pid_gains(n=10):
    pid_list = []
    for _ in range(n):
        Kp = round(random.uniform(0, 50), 3)
        Ki = round(random.uniform(0, 10), 3)
        Kd = round(random.uniform(0, 5) , 3)
        pid_list.append([Kp, Ki, Kd])
    return pid_list

initial_set = generate_random_pid_gains(10)

start_time = time.time()

auto_tunning = pid_auto_tunning.PIDAutoTunning(dc_motor_model, goodhart_gains=[0.01, 0.01, 0.98])
parcial_result = auto_tunning.tunning(initial_set, False)
parcial_result = sorted(parcial_result, key=lambda x: x[1])

pid_gains = parcial_result[0][0]
    
n = int((1 / (auto_tunning.ts_ms / 1000.0))*auto_tunning.tf + 1)
time_vector = np.linspace(0, auto_tunning.tf, n)

auto_tunning.set_save_figs(True)

result = minimize(
    auto_tunning.run_pid,
    pid_gains,
    args=(time_vector, np.cos(time_vector)),
    method='Nelder-Mead',
    options={
        "maxiter": 100,
        "maxfev": 120,    
        "fatol": 0.01,       
        "xatol": 0.01        
    }
)

end_time = time.time()
elapsed = end_time - start_time

print(f"\nTempo total de execução: {elapsed:.3f} segundos")

print("Optimization successful:", result.success)
print("Message:", result.message)
print("Optimal parameters (x):", result.x)
print("Minimum function value (fun):", result.fun)
print("Number of function evaluations (nfev):", result.nfev)