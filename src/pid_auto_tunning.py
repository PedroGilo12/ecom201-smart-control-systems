import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import random
import time
import pandas as pd
import os
from concurrent.futures import ProcessPoolExecutor

class PIDAutoTunning():

    def __init__(self, model, tf = 6.0, ts_ms = 0.01, goodhart_gains = [0.33, 0.33, 0.34], save_folder="results"):
        self.model = model
        self.tf = tf
        self.ts_ms = ts_ms

        self.integral_erro = 0
        self.previous_erro = 0

        self.upper = 255
        self.lower = -255

        self.goodhart_gains = goodhart_gains
        self.save_folder = save_folder

        self.save_figs = False

        os.makedirs(self.save_folder, exist_ok=True)

    def define_control_bound(self, upper, lower):
        self.upper = upper
        self.lower = lower

    @staticmethod
    def _run_single_test(args):
        self_obj, gain = args
        return gain, self_obj.run_pid(gain)

    def tunning(self, pid_gains, save_figs = False):
        tasks = [(self, gain) for gain in pid_gains]
        self.save_figs = save_figs

        start_time = time.time()

        result = []
        with ProcessPoolExecutor() as ex:
            for gain, goodhart in ex.map(PIDAutoTunning._run_single_test, tasks):
                result.append([gain, goodhart])

        end_time = time.time()
        exec_time = end_time - start_time
        print(f"\nTempo total do tunning: {exec_time:.3f} segundos\n")

        return result
    
    def set_save_figs(self, save_figs):
        self.save_figs = save_figs

    def run_pid(self, pid_gains, time_vector = None, ref = None):

        Kp, Ki, Kd = pid_gains

        states0 = [0]
        n = int((1 / (self.ts_ms / 1000.0))*self.tf + 1)

        if time_vector is None:
            time_vector = np.linspace(0, self.tf, n)

        t_sim_step = time_vector[1] - time_vector[0]

        if ref is None:
            arr1 = np.full((int(time_vector.shape[0]/2),), 0)
            arr2 = np.full((int(time_vector.shape[0]/2),), 5)
            ref = np.concatenate((arr1, arr2))

        states = np.zeros((n-1, 2))

        tau = states0[0]
        initial_time = time.time()
        t_counter = 0

        SC_list = []
        erro_list = []

        for i in range(n-1):

            erro = ref[i] - tau
            self.integral_erro += erro * t_sim_step
            self.derivada_erro = (erro - self.previous_erro) / t_sim_step

            Up = Kp * erro
            Ui = Ki * self.integral_erro
            Ud = Kd * self.derivada_erro

            dc_volts = Up + Ui - Ud

            dc_volts = min(max(dc_volts, self.lower), self.upper)

            SC_list.append(dc_volts)
            erro_list.append(erro)

            out_states = odeint(self.model, tau, [0.0, self.tf/n], args=(dc_volts,))
            tau = out_states[-1, 0]
            self.previous_erro = erro

            states[i, 0] = tau
            states[i, 1] = dc_volts

            if i >= t_counter * int((n-1)/10):
                #print("Simulation at {}%".format(t_counter*10))
                t_counter += 1

        SC_arr = np.array(SC_list)
        erro_arr = np.array(erro_list)

        goodhart_e1 = np.mean(SC_arr)
        goodhart_e2 = np.mean((SC_arr - goodhart_e1)**2)
        goodhart_e3 = np.mean(erro_arr**2)

        goodhart_index = (
            self.goodhart_gains[0] * goodhart_e1 +
            self.goodhart_gains[1] * goodhart_e2 +
            self.goodhart_gains[2] * goodhart_e3
        )

        if self.save_figs:
            plt.rcParams['axes.grid'] = True
            fig = plt.figure(figsize=(10,6))

            plt.subplot(2,1,1)
            plt.plot(time_vector[:-1], ref[:-1], 'k--', linewidth=3, label="ref")
            plt.plot(time_vector[:-1], states[:,0], 'r', linewidth=2, label="tau")
            plt.ylabel('tau [Nm]')
            plt.legend()

            plt.subplot(2,1,2)
            plt.plot(time_vector[:-1], states[:,1], 'b', linewidth=2, label="dc_volts")
            plt.ylabel('dc_volts [V]')
            plt.legend()

            fig.suptitle(
                f"PID: Kp={Kp}, Ki={Ki}, Kd={Kd} | Goodhart = {goodhart_index:.5f}",
                fontsize=12
            )

            filename = f"ghi_{round(goodhart_index, 6)}.png"
            filepath = os.path.join(self.save_folder, filename)

            plt.savefig(filepath)
            plt.close(fig)

            print(f"Gráfico salvo em: {filepath}")
        print(f"Execution for Kp {Kp} Ki {Ki} Kd {Kd} terminate with goodhart: {goodhart_index:.5f}.")

        return goodhart_index
