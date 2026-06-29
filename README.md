# SLAM e Localização com AMCL — Hector SLAM vs. Gmapping (LaR UFBA, Husky UGV)

Atividade da disciplina **Tópicos Especiais em Engenharia Elétrica IV** — UFBA.

O projeto compara dois métodos de SLAM (**Hector SLAM** e **Gmapping**) na geração de mapas do ambiente simulado do LaR, seguido de localização com **AMCL** sobre cada mapa, comparando a pose estimada com o *ground truth* fornecido pelo Gazebo (`/gazebo/model_states`).

Este repositório usa como base o pacote ROS [`lar_gazebo`](https://github.com/lar-deeufba/lar_gazebo) do laboratório LaR/UFBA (ambiente simulado, modelos, mundo, integração com Husky). Sobre essa base foram adicionados os arquivos desta atividade: `launch/amcl.launch`, `scripts/captura_poses.py`, `scripts/calcular_metricas.py`, os mapas gerados e os resultados.

## Ambiente

- ROS Noetic + Gazebo Classic 11
- Robô: Husky UGV (laser frontal `/front/scan`)
- Execução via Docker (`docker-compose.yml` neste repositório)

## Estrutura do repositório

```
launch/
  lar_world.launch       -- sobe apenas o mundo do LaR no Gazebo
  lar_husky.launch       -- sobe Gazebo + Husky (opcionalmente com hector_slam:=true)
  hector_slam.launch     -- SLAM com Hector
  gmapping.launch        -- SLAM com gmapping
  amcl.launch             -- AMCL + map_server, parametrizado por mapa (criado para esta atividade)
scripts/
  run_husky.sh
  entrar.sh               -- acessa o container em execução sem precisar do ID manualmente
  captura_poses.py        -- grava ground truth + pose AMCL em CSV (criado para esta atividade)
  calcular_metricas.py    -- calcula RMSE, erro final, orientação, estabilidade (criado para esta atividade)
maps/
  mapa_hector.pgm / .yaml
  mapa_gmapping.pgm / .yaml
resultados/
  poses_mapa_hector.csv
  poses_mapa_hector_original.csv   -- captura bruta antes do corte de trechos com robô parado (ver Metodologia)
  poses_mapa_gmapping.csv
  erro_posicao_mapa_hector.csv
  erro_posicao_mapa_gmapping.csv
  erro_ao_longo_do_tempo.png
docker/
  Dockerfile.noetic
  entrypoint.sh
```

Os demais diretórios (`models/`, `worlds/`, `maps/lab_robotica_06mai2019.*`, `april_tags/`, `husky_urdf_extras/`, `husky_accessories.sh`, `config/hector_slam.rviz`) pertencem ao pacote base do laboratório, necessários para o `catkin build`/`roslaunch` funcionarem. Não foram alterados nesta atividade.

## Como executar

### 1. Build da imagem Docker

```bash
docker compose build
```

### 2. Subir o ambiente

```bash
./scripts/run_husky.sh gui:=false
```

A GUI 3D do Gazebo fica desligada por padrão. Em testes, ela causou sobrecarga de CPU (>400%) suficiente para introduzir uma defasagem real de ~8s entre o timestamp do `/front/scan` e o `/clock` simulado, esgotando o buffer fixo do `tf::MessageFilter` usado pelo `slam_gmapping` e travando a publicação do `/map` (detalhes na seção de Discussão). Use o RViz para navegação visual em vez da GUI do Gazebo.

### 3. Gerar os mapas

Em outro terminal, acesse o container:
```bash
./scripts/entrar.sh
```

**Gmapping:**
```bash
roslaunch lar_gazebo gmapping.launch
# em outro terminal:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kb_teleop/cmd_vel
# ative o display "Map" no RViz desde o início da exploração, para acompanhar a cobertura em tempo real
rosrun map_server map_saver -f /ws/src/lar_gazebo/maps/mapa_gmapping
```

**Hector SLAM:**
```bash
./scripts/run_husky.sh gui:=false hector_slam:=true
# teleoperar e salvar mapa_hector da mesma forma
rosrun map_server map_saver -f /ws/src/lar_gazebo/maps/mapa_hector
```

Sobre o comando de teleoperação: use sempre `cmd_vel:=/kb_teleop/cmd_vel`, nunca `/husky_velocity_controller/cmd_vel` diretamente. O nó `/twist_mux` arbitra entre múltiplas fontes (teclado, joystick) e só ele publica na saída final do Husky. Publicar direto na saída cria dois publishers concorrentes no mesmo tópico, e o teclado simplesmente perde — o robô parece não responder mesmo sem nenhum erro nos logs.

### 4. Rodar o AMCL e capturar dados

Para cada mapa:
```bash
roslaunch lar_gazebo amcl.launch map_file:=/ws/src/lar_gazebo/maps/mapa_hector.yaml
```

No RViz, usar **2D Pose Estimate** (clique-arrastar-soltar, não um clique simples) para indicar a pose inicial real do robô.

Antes de capturar os dados, vale confirmar a calibração:
```bash
# pose real do robô no mundo (ground truth) -- ANTES do Pose Estimate
rostopic echo /gazebo/model_states -n 1
```
Anote `x`, `y` do modelo `husky`. Depois do Pose Estimate, confirme que a pose do AMCL está de fato próxima do ground truth:
```bash
rostopic echo /amcl_pose -n 1
```
A diferença deve ser de poucos centímetros. Se for da ordem de metros, o Pose Estimate foi feito no ponto errado do mapa — refaça antes de capturar. Foi exatamente isso que aconteceu nesta execução (ver Limitações).

Capturando:
```bash
python3 scripts/captura_poses.py mapa_hector
# teleoperar cobrindo o ambiente; Ctrl+C na captura ao terminar
```

Repetir substituindo por `mapa_gmapping`, refazendo o 2D Pose Estimate (a pose anterior não vale para o novo mapa).

Para validar o CSV antes de aceitar os dados, o critério correto não é o percentual de linhas repetidas no arquivo todo — uma taxa de ~99% é esperada e normal, já que o ground truth (`/gazebo/model_states`) publica a ~100Hz enquanto o AMCL atualiza a ~0.9Hz (dado `update_min_d=0.2`, `update_min_a=0.2`). O que importa é a maior sequência contínua de repetição em segundos: valores na faixa de 1–4s são saudáveis, sequências de 10s ou mais indicam robô parado por algum problema operacional naquele trecho.

### 5. Calcular métricas

```bash
python3 scripts/calcular_metricas.py resultados/poses_mapa_hector.csv resultados/poses_mapa_gmapping.csv
```

Gera no terminal as métricas de cada execução e a comparação direta, além de:
- `erro_posicao_<mapa>.csv` — série temporal do erro
- `erro_ao_longo_do_tempo.png` — gráfico comparativo de erro de posição/orientação

## Metodologia — nota sobre `poses_mapa_hector.csv`

A captura original do Hector (`poses_mapa_hector_original.csv`, preservada no repositório para auditoria) tinha dois trechos longos de robô parado por interrupção operacional, sem relação com AMCL ou SLAM: ~25s no início e ~15s no fim, somando boa parte do arquivo. O `poses_mapa_hector.csv` usado nas métricas finais é a versão recortada para o intervalo em que o robô estava de fato em movimento contínuo.

## Resultados

### Métricas quantitativas

| Métrica | Hector SLAM | Gmapping |
|---|---|---|
| RMSE de posição (m) | 5,74 | 6,22 |
| Desvio padrão do erro de posição (m) | 0,2300 | 1,1087 |
| Erro de orientação médio (graus) | 6,04 | 7,64 |

![Erro ao longo do tempo](lar_gazebo/resultados/erro_ao_longo_do_tempo.png)

Os valores de RMSE acima estão distorcidos por um offset de calibração inicial do AMCL e não representam a precisão real de localização dos dois métodos (explicação completa na seção de Limitações). Por isso o desvio padrão foi usado como métrica principal de comparação.

### Análise qualitativa dos mapas

| Critério | Hector SLAM | Gmapping |
|---|---|---|
| Completude | Perímetro exterior bem definido, cantos retos e o corredor apresentando limitações | Contorno fechado, cantos bem definidos após uma exploração mais longa e detalhada |
| Distorções | Mínimas; estrutura geral apresentou fidelidade ao ambiente analisado | Sem o artefato de "leque" grande presente em tentativas iniciais em que a exploração foi mais curta |
| Paredes desalinhadas | Não observadas de uma maneira tão significativa | Não observadas ao "pé da letra" na versão final |
| Obstáculos falsos | Obstáculos centrais (mesas) com bordas apresentando nitidez, sem a presença de ruído espúrio relevante | Bordas dos obstáculos centrais com uma difusão ligeiramente mais visível que no Hector |
| Regiões desconhecidas | Pequeno artefato de incerteza próximo a uma abertura, em que tem-se extensão limitada | Dependência diretamente da cobertura da exploração; melhorou significativamente com navegação mais longa e também orientada |
| Qualidade da localização (AMCL) | Maior estabilidade: desvio padrão do erro em torno de ~4,8x menor que o Gmapping | Maior variabilidade na estimativa de pose ao longo da trajetória |

## Discussão

### Melhor mapa

O Hector SLAM produziu o mapa mais completo e com menos artefatos visuais, mesmo depois do Gmapping ter sido regenerado com exploração mais longa e cuidadosa (display `Map` ativo no RViz desde o início, pra guiar a cobertura). O Hector não depende de uma cadeia de TF sincronizada para o scan matching — ele casa o laser direto contra o mapa em construção. Isso também explica a maior consistência dele durante a geração do mapa.

### Melhor localização

O RMSE e a estabilidade apresentam uma seguinte discussão: O RMSE de posição (5,74 m para Hector, 6,22 m para Gmapping) e o erro de posição final não devem ser lidos como a precisão real da localização, levado ao fato de que estão distorcidos por um offset de calibração identificado durante a execução. Assim, a comparação usa o desvio padrão do erro de posição, sendo responsável por medir a variabilidade em torno da própria referência da trajetória, e, não é afetado por um offset constante somado a todas as amostras.

Em função disso, o Hector permitiu localização bem mais estável: desvio padrão de 0,23 m contra 1,11 m do Gmapping, quase 5 vezes menor. O erro de orientação médio segue a mesma tendência (6,04° vs. 7,64°). Provavelmente o mapa do Hector forneceu uma estrutura mais consistente para o *likelihood field* do AMCL se ancorar a cada atualização, resultando em menor variância na pose estimada. Pode ter relação também com a diferença de área de varredura entre os dois mapas: `mapa_gmapping.yaml` tem origem `[-50.0, -50.0]` e cobre uma área bem maior que `mapa_hector.yaml`, origem `[-6.425, -6.425]`.

### Acerca da sobrecarga computacional

Durante a geração de mapas, o gmapping apresentou falha de publicação do `/map` causada por sobrecarga de CPU da GUI 3D do Gazebo. Essa sobrecarga introduzia um atraso real de ~8s entre o timestamp do `/front/scan` e o `/clock` simulado, suficiente para esgotar o buffer fixo (5 mensagens) do `tf::MessageFilter` usado internamente pelo `slam_gmapping`. O Hector não apresentou esse problema por não depender de TF sincronizada para o scan matching. Essa diferença de confiabilidade frente a atraso/sobrecarga computacional é relevante para a comparação entre os dois algoritmos, independente da qualidade final dos mapas.

## Limitações conhecidas

O "2D Pose Estimate" inicial foi posicionado, nas duas execuções, em um ponto do mapa que não corresponde exatamente à posição real do robô no Gazebo. A diferença entre a pose estimada pelo AMCL e o ground truth fica praticamente constante do começo ao fim da trajetória, em vez de crescer com o tempo — o que indica offset de calibração, não drift. O filtro de partículas convergiu normalmente e se manteve estável, mas em torno de uma referência inicial deslocada.

Esse tipo de erro não gera nenhum aviso nos logs do ROS — o AMCL converge normalmente e parece saudável, só que com a referência inteira deslocada. Por isso, qualquer RMSE de posição na faixa de metros mostra a necessidade de verificar a calibração do Pose Estimate antes de aceitar os números como representativos da localização real, exatamente a questão observada.

## Autor

Lucas Fialho — UFBA, Tópicos Especiais em Engenharia Elétrica IV
