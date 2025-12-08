import random
import numpy as np
import pid_auto_tunning
import time


start_time = time.time()

a = 1
k = 1

def dc_motor_model(x1_m, t, u):
    dx1_m = -a*k*x1_m + k*u
    return dx1_m

auto_tunning = pid_auto_tunning.PIDAutoTunning(dc_motor_model, goodhart_gains=[0.01, 0.01, 0.98])

def algoritmo_genetico(N=20, M=40, F=10, tol=1e-3, t=3):

    bounds = [(0, 50), (0, 20), (0, 1)]  # Kp, Ki, Kd

    crom = []

    for _ in range(N):
        c = [random.uniform(bounds[i][0], bounds[i][1]) for i in range(3)]
        crom.append(c)

    P = auto_tunning.tunning(crom, save_figs=False)
    P = sorted(P, key=lambda x: x[1])
    k = N

    while True:

        filhos = []   

        for _ in range(F):

            r1 = random.random() ** 2
            r2 = random.random() ** 2

            m = int(r1 * N)
            f = int(r2 * N)

            crom_m = P[m][0][:]
            crom_f = P[f][0][:]

            # Mutação
            idx_m = random.randint(0, 2)
            idx_f = random.randint(0, 2)
            crom_m[idx_m] += random.uniform(-1, 1)
            crom_f[idx_f] += random.uniform(-1, 1)

            # Limites
            for i in range(3):
                crom_m[i] = max(bounds[i][0], min(bounds[i][1], crom_m[i]))
                crom_f[i] = max(bounds[i][0], min(bounds[i][1], crom_f[i]))

            # Crossover
            corte = random.randint(1, 2)
            filho1 = crom_f[:corte] + crom_m[corte:]
            filho2 = crom_m[:corte] + crom_f[corte:]

            filhos.append(filho1)
            filhos.append(filho2)

        # ------------------------------------------------------
        # Avaliar todos os filhos F*2 de uma vez
        # ------------------------------------------------------
        P_child = auto_tunning.tunning(filhos, save_figs=False)

        # Inserir
        for ind in P_child:
            P.append(ind)

        k += len(P_child)

        # Ordenar
        P = sorted(P, key=lambda x: x[1])

        # Eliminação
        if k >= M:
            P = P[:M]
            k = M

        # Critério de parada
        if P[t][1] - P[0][1] < tol:
            return P[0]

best = algoritmo_genetico(N=20, M=40, tol=1e-3, t=3)
auto_tunning.set_save_figs(True)
auto_tunning.run_pid(best[0])

end_time = time.time()
elapsed = end_time - start_time

print(f"\nTempo total de execução: {elapsed:.3f} segundos")

print("Melhor solução encontrada:")
print("Kp =", best[0][0])
print("Ki =", best[0][1])
print("Kd =", best[0][2])
print("Custo =", best[1])