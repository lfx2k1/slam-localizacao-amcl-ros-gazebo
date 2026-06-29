#!/usr/bin/env python3
"""
calcular_metricas.py

Lê um CSV gerado por captura_poses.py e calcula as métricas quantitativas
pedidas na atividade: erro de posição instantâneo (série completa), RMSE de
posição, erro de posição final, erro de orientação e estabilidade (variação
do erro ao longo da trajetória).

Uso:
    python3 calcular_metricas.py poses_mapa_hector.csv
    python3 calcular_metricas.py poses_mapa_gmapping.csv

Para comparar os dois de uma vez (recomendado, gera também o gráfico
comparativo de erro ao longo do tempo):
    python3 calcular_metricas.py poses_mapa_hector.csv poses_mapa_gmapping.csv

Saídas geradas por execução, além do texto no terminal:
    erro_posicao_<nome>.csv   -- série temporal do erro de posição instantâneo
    erro_ao_longo_do_tempo.png -- gráfico comparativo (se 2 arquivos forem passados)

Requer matplotlib para o gráfico (pip3 install matplotlib --break-system-packages).
Se não estiver disponível, o script ainda funciona e só pula a parte do gráfico.
"""

import sys
import math
import csv

try:
    import matplotlib
    matplotlib.use("Agg")  # não precisa de display gráfico (funciona dentro do container)
    import matplotlib.pyplot as plt
    TEM_MATPLOTLIB = True
except ImportError:
    TEM_MATPLOTLIB = False


def carregar_csv(caminho):
    linhas = []
    with open(caminho, newline="") as f:
        leitor = csv.DictReader(f)
        for linha in leitor:
            linhas.append({k: float(v) for k, v in linha.items()})
    return linhas


def diferenca_angular(a, b):
    """Diferença angular a-b, normalizada para (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


def calcular_metricas(linhas, nome_execucao):
    n = len(linhas)
    if n == 0:
        print(f"[{nome_execucao}] CSV vazio, nada para calcular.")
        return None, None

    tempos = []
    erros_posicao = []
    erros_yaw = []

    t0 = linhas[0]["tempo_ros"]

    for linha in linhas:
        dx = linha["amcl_x"] - linha["gt_x"]
        dy = linha["amcl_y"] - linha["gt_y"]
        erro_pos = math.hypot(dx, dy)
        erro_yaw = abs(diferenca_angular(linha["amcl_yaw"], linha["gt_yaw"]))

        tempos.append(linha["tempo_ros"] - t0)  # tempo relativo ao início da captura
        erros_posicao.append(erro_pos)
        erros_yaw.append(erro_yaw)

    # RMSE de posição
    rmse_posicao = math.sqrt(sum(e ** 2 for e in erros_posicao) / n)

    # Erro de posição final (última amostra)
    erro_posicao_final = erros_posicao[-1]
    erro_yaw_final = erros_yaw[-1]

    # Erro médio de orientação
    erro_yaw_medio = sum(erros_yaw) / n

    # Estabilidade: desvio padrão do erro de posição ao longo do tempo
    media_erro_pos = sum(erros_posicao) / n
    variancia_erro_pos = sum((e - media_erro_pos) ** 2 for e in erros_posicao) / n
    desvio_padrao_erro_pos = math.sqrt(variancia_erro_pos)

    resultado = {
        "nome_execucao": nome_execucao,
        "n_amostras": n,
        "rmse_posicao_m": rmse_posicao,
        "erro_posicao_final_m": erro_posicao_final,
        "erro_yaw_final_rad": erro_yaw_final,
        "erro_yaw_final_deg": math.degrees(erro_yaw_final),
        "erro_yaw_medio_rad": erro_yaw_medio,
        "erro_yaw_medio_deg": math.degrees(erro_yaw_medio),
        "desvio_padrao_erro_posicao_m": desvio_padrao_erro_pos,
        "erro_posicao_min_m": min(erros_posicao),
        "erro_posicao_max_m": max(erros_posicao),
    }

    serie_temporal = {
        "tempos": tempos,
        "erros_posicao": erros_posicao,
        "erros_yaw_deg": [math.degrees(e) for e in erros_yaw],
    }

    return resultado, serie_temporal


def salvar_serie_temporal_csv(serie, nome_execucao):
    caminho = f"erro_posicao_{nome_execucao}.csv"
    with open(caminho, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tempo_s", "erro_posicao_m", "erro_orientacao_deg"])
        for t, ep, ey in zip(serie["tempos"], serie["erros_posicao"], serie["erros_yaw_deg"]):
            w.writerow([t, ep, ey])
    print(f"Série temporal de erro salva em: {caminho}")


def imprimir_resultado(r):
    print(f"\n=== Métricas: {r['nome_execucao']} ===")
    print(f"Amostras consideradas:        {r['n_amostras']}")
    print(f"RMSE de posição:              {r['rmse_posicao_m']:.4f} m")
    print(f"Erro de posição final:        {r['erro_posicao_final_m']:.4f} m")
    print(f"Erro de orientação final:     {r['erro_yaw_final_deg']:.2f} graus "
          f"({r['erro_yaw_final_rad']:.4f} rad)")
    print(f"Erro de orientação médio:     {r['erro_yaw_medio_deg']:.2f} graus "
          f"({r['erro_yaw_medio_rad']:.4f} rad)")
    print(f"Desvio padrão erro posição:   {r['desvio_padrao_erro_posicao_m']:.4f} m  "
          f"(estabilidade -- menor é melhor)")
    print(f"Erro de posição (min / max):  {r['erro_posicao_min_m']:.4f} m / "
          f"{r['erro_posicao_max_m']:.4f} m")


def gerar_grafico_comparativo(series_e_nomes):
    """series_e_nomes: lista de tuplas (serie_temporal, nome_execucao)."""
    if not TEM_MATPLOTLIB:
        print("\n[aviso] matplotlib não está instalado -- gráfico não gerado.")
        print("Para gerar: pip3 install matplotlib --break-system-packages")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=False)

    for serie, nome in series_e_nomes:
        ax1.plot(serie["tempos"], serie["erros_posicao"], label=nome)
        ax2.plot(serie["tempos"], serie["erros_yaw_deg"], label=nome)

    ax1.set_ylabel("Erro de posição (m)")
    ax1.set_title("Erro de posição ao longo da trajetória (AMCL vs. ground truth)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.set_xlabel("Tempo (s, relativo ao início da captura)")
    ax2.set_ylabel("Erro de orientação (graus)")
    ax2.set_title("Erro de orientação ao longo da trajetória")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    caminho = "erro_ao_longo_do_tempo.png"
    fig.savefig(caminho, dpi=150)
    print(f"\nGráfico comparativo salvo em: {caminho}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    resultados = []
    series_e_nomes = []

    for caminho in sys.argv[1:]:
        nome_execucao = caminho.replace("poses_", "").replace(".csv", "")
        linhas = carregar_csv(caminho)
        r, serie = calcular_metricas(linhas, nome_execucao)
        if r:
            resultados.append(r)
            imprimir_resultado(r)
            salvar_serie_temporal_csv(serie, nome_execucao)
            series_e_nomes.append((serie, nome_execucao))

    if len(resultados) == 2:
        print("\n=== Comparação direta ===")
        a, b = resultados
        print(f"{'Métrica':<30} {a['nome_execucao']:>15} {b['nome_execucao']:>15}")
        print(f"{'RMSE posição (m)':<30} {a['rmse_posicao_m']:>15.4f} {b['rmse_posicao_m']:>15.4f}")
        print(f"{'Erro posição final (m)':<30} {a['erro_posicao_final_m']:>15.4f} {b['erro_posicao_final_m']:>15.4f}")
        print(f"{'Erro orientação final (graus)':<30} {a['erro_yaw_final_deg']:>15.2f} {b['erro_yaw_final_deg']:>15.2f}")
        print(f"{'Desvio padrão erro (m)':<30} {a['desvio_padrao_erro_posicao_m']:>15.4f} {b['desvio_padrao_erro_posicao_m']:>15.4f}")

        melhor_rmse = a["nome_execucao"] if a["rmse_posicao_m"] < b["rmse_posicao_m"] else b["nome_execucao"]
        melhor_estab = (
            a["nome_execucao"]
            if a["desvio_padrao_erro_posicao_m"] < b["desvio_padrao_erro_posicao_m"]
            else b["nome_execucao"]
        )
        print(f"\nMenor RMSE de posição: {melhor_rmse}")
        print(f"Maior estabilidade (menor desvio padrão): {melhor_estab}")

        gerar_grafico_comparativo(series_e_nomes)
    elif len(series_e_nomes) == 1:
        gerar_grafico_comparativo(series_e_nomes)


if __name__ == "__main__":
    main()
