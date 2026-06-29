#!/usr/bin/env python3
"""
captura_poses.py

Grava, em paralelo, a pose ground truth do Husky (via /gazebo/model_states)
e a pose estimada pelo AMCL (via /amcl_pose) em um único CSV, sincronizadas
por timestamp de chegada da mensagem de ground truth.

Uso (dentro do container, com Gazebo + AMCL já rodando):

    python3 captura_poses.py mapa_hector
    python3 captura_poses.py mapa_gmapping

O nome passado como argumento só é usado para nomear o arquivo de saída
(ex.: poses_mapa_hector.csv), não tem efeito nenhum no ROS.

Pare com Ctrl+C quando achar que coletou dados suficientes (recomendado:
pelo menos 1-2 minutos de robô em movimento, cobrindo trechos variados
do ambiente, não só ficando parado).
"""

import csv
import math
import sys
import signal

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped


# Nome do modelo do Husky na lista publicada por /gazebo/model_states.
# Confirme o nome real com: rostopic echo /gazebo/model_states -n 1
# (normalmente é "husky" — ajuste aqui se for diferente no seu .world)
NOME_MODELO_HUSKY = "husky"


def yaw_do_quaternion(q):
    """Extrai yaw (rotação em torno de Z) de um quaternion geometry_msgs/Quaternion."""
    seno = 2.0 * (q.w * q.z + q.x * q.y)
    coseno = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(seno, coseno)


class CapturaPoses:
    def __init__(self, caminho_csv):
        self.ultima_pose_amcl = None  # (x, y, yaw) ou None se ainda não chegou nenhuma
        self.linhas_gravadas = 0

        self.arquivo = open(caminho_csv, "w", newline="")
        self.writer = csv.writer(self.arquivo)
        self.writer.writerow([
            "tempo_ros",
            "gt_x", "gt_y", "gt_yaw",
            "amcl_x", "amcl_y", "amcl_yaw",
        ])

        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl, queue_size=10
        )
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.callback_ground_truth, queue_size=10
        )

        rospy.loginfo("Gravando em: %s", caminho_csv)
        rospy.loginfo("Aguardando primeira mensagem de /amcl_pose...")

    def callback_amcl(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_do_quaternion(msg.pose.pose.orientation)
        self.ultima_pose_amcl = (p.x, p.y, yaw)

    def callback_ground_truth(self, msg):
        # Cada chegada de /gazebo/model_states dispara uma linha no CSV,
        # usando a última pose do AMCL conhecida até o momento (sincronização
        # simples por "amostra mais recente disponível").
        if self.ultima_pose_amcl is None:
            return  # ainda não há estimativa do AMCL -- não grava ainda

        try:
            idx = msg.name.index(NOME_MODELO_HUSKY)
        except ValueError:
            rospy.logwarn_throttle(
                5.0,
                "Modelo '%s' não encontrado em /gazebo/model_states. "
                "Nomes disponíveis: %s",
                NOME_MODELO_HUSKY, msg.name,
            )
            return

        pose_gt = msg.pose[idx]
        gt_x = pose_gt.position.x
        gt_y = pose_gt.position.y
        gt_yaw = yaw_do_quaternion(pose_gt.orientation)

        amcl_x, amcl_y, amcl_yaw = self.ultima_pose_amcl
        tempo = rospy.Time.now().to_sec()

        self.writer.writerow([tempo, gt_x, gt_y, gt_yaw, amcl_x, amcl_y, amcl_yaw])
        self.linhas_gravadas += 1

        if self.linhas_gravadas % 50 == 0:
            rospy.loginfo("Linhas gravadas: %d", self.linhas_gravadas)

    def fechar(self):
        self.arquivo.close()
        rospy.loginfo("Arquivo fechado. Total de linhas: %d", self.linhas_gravadas)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    nome_execucao = sys.argv[1]
    caminho_csv = f"poses_{nome_execucao}.csv"

    rospy.init_node("captura_poses", anonymous=True)
    captura = CapturaPoses(caminho_csv)

    def ao_encerrar(sig, frame):
        captura.fechar()
        rospy.signal_shutdown("Ctrl+C")

    signal.signal(signal.SIGINT, ao_encerrar)

    rospy.spin()


if __name__ == "__main__":
    main()
