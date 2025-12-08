import random
import numpy as np
import pid_auto_tunning

a = 1
k = 1

def dc_motor_model(x1_m, t, u):
    """Modelo de primeira ordem de um Motor DC."""
    dx1_m = -a*k*x1_m + k*u
    return dx1_m

auto_tunning = pid_auto_tunning.PIDAutoTunning(dc_motor_model)

def algoritmo_pso(N=20, M=100, W=0.7, C1=1.5, C2=1.5, tol=1e-3):
    """
    Algoritmo de Otimização por Enxame de Partículas (PSO) para sintonia PID.

    N: Número de partículas (tamanho do enxame).
    M: Número máximo de iterações.
    W: Coeficiente de Inércia.
    C1: Coeficiente de atração para pbest (melhor posição pessoal).
    C2: Coeficiente de atração para gbest (melhor posição global).
    tol: Critério de parada de tolerância (diferença no gbest).
    """

    bounds = np.array([(0, 100), (0, 50), (0, 10)])
    dim = len(bounds) 

    particulas = []
    velocidades = []
    pbest_posicoes = []
    pbest_custos = []
    
    gbest_posicao = None
    gbest_custo = np.inf

    for _ in range(N):
        pos = [random.uniform(bounds[i][0], bounds[i][1]) for i in range(dim)]
        particulas.append(pos)
        
        vel = [random.uniform(-(bounds[i][1] - bounds[i][0])/10, (bounds[i][1] - bounds[i][0])/10) for i in range(dim)]
        velocidades.append(vel)
        
        pbest_posicoes.append(list(pos))
        pbest_custos.append(np.inf)

    for iteracao in range(M):

        avaliacao = auto_tunning.tunning(particulas, save_figs=False)
        custos_atuais = [res[1] for res in avaliacao]
        
        for i in range(N):
            custo_atual = custos_atuais[i]
            
            if custo_atual < pbest_custos[i]:
                pbest_custos[i] = custo_atual
                pbest_posicoes[i] = particulas[i][:]
            
            if custo_atual < gbest_custo:
                if gbest_custo - custo_atual < tol and iteracao > 0:
                    print(f"Critério de tolerância atingido na iteração {iteracao}.")
                    return gbest_posicao, gbest_custo
                
                gbest_custo = custo_atual
                gbest_posicao = particulas[i][:] 
                print(f"Nova Gbest na iteração {iteracao}: Custo = {gbest_custo:.5f}, Pos = {gbest_posicao}")

        for i in range(N):
            r1 = random.uniform(0, 1)
            r2 = random.uniform(0, 1)
            
            nova_velocidade = []
            nova_posicao = []
            
            for d in range(dim):
                inercia = W * velocidades[i][d]
                cognitivo = C1 * r1 * (pbest_posicoes[i][d] - particulas[i][d])
                social = C2 * r2 * (gbest_posicao[d] - particulas[i][d])
                
                vel_d = inercia + cognitivo + social
                nova_velocidade.append(vel_d)

                pos_d = particulas[i][d] + vel_d
                
                pos_d = max(bounds[d][0], min(bounds[d][1], pos_d))
                nova_posicao.append(pos_d)

            velocidades[i] = nova_velocidade
            particulas[i] = nova_posicao

        if iteracao == M - 1:
             print("Máximo de iterações atingido.")


    return gbest_posicao, gbest_custo

if __name__ == '__main__':
    best_pos, best_cost = algoritmo_pso(N=10, M=50, W=0.7, C1=1.5, C2=1.5, tol=1e-5)

    if best_pos is not None:
        auto_tunning.set_save_figs(True)
        auto_tunning.run_pid(best_pos)

        print("\nMelhor solução encontrada pelo PSO:")
        print(f"Kp = {best_pos[0]:.4f}")
        print(f"Ki = {best_pos[1]:.4f}")
        print(f"Kd = {best_pos[2]:.4f}")
        print(f"Custo (Goodhart) = {best_cost:.6f}")
    else:
        print("Nenhuma solução válida foi encontrada.")