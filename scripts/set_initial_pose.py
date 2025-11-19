#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # 1. Cria o publisher no tópico /initialpose
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # 2. Cria um timer para publicar a mensagem uma vez (ou periodicamente, se necessário)
        # Vamos publicar apenas uma vez 1 segundo após o nó iniciar.
        self.timer_ = self.create_timer(1.0, self.publish_initial_pose_once)
        
        self.get_logger().info('Initial Pose Publisher Node iniciado. Preparando para enviar a pose inicial...')
        
        # Flag para garantir que a mensagem só seja enviada uma vez
        self.pose_sent = False

    def publish_initial_pose_once(self):
        if self.pose_sent:
            return

        # Cria a mensagem completa PoseWithCovarianceStamped
        initial_pose_msg = PoseWithCovarianceStamped()

        # --- HEADER (Cabeçalho) ---
        # Define o frame de referência como 'map'
        initial_pose_msg.header.frame_id = 'map'
        # O timestamp é preenchido automaticamente ao chamar self.get_clock().now().to_msg()
        # Nota: O AMCL ou Nav2 geralmente preenchem o timestamp automaticamente se você não o definir,
        # mas é boa prática preenchê-lo explicitamente em um nó.
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

        # --- POSE (Posição e Orientação) ---
        pose_with_cov = PoseWithCovariance()
        
        # Posição (x: 1.0, y: 1.0, z: 0.0)
        pose_with_cov.pose.position = Point(x=1.0, y=1.0, z=0.0)
        
        # Orientação (x: 0.0, y: 0.0, z: 0.0, w: 1.0 - Orientação 0)
        pose_with_cov.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # --- COVARIANCE (Matriz 6x6 achatada em um array de 36) ---
        # Usamos zeros, indicando incerteza mínima ou um valor padrão (a incerteza real é ajustada pelo AMCL)
        # O AMCL usa os valores da covariância para espalhar as partículas.
        # Os valores típicos não-zero para localização inicial são nas diagonais (0, 7, 14, 21, 28, 35)
        
        # Exemplo com valores típicos para incerteza inicial (Diagonal)
        # [x, y, z, roll, pitch, yaw]
        pose_with_cov.covariance = [
             0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 00.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        initial_pose_msg.pose = pose_with_cov
        
        # 3. Publica a mensagem
        self.publisher_.publish(initial_pose_msg)
        self.pose_sent = True
        
        self.get_logger().info('>>> Pose inicial (x=1.0, y=1.0) publicada com sucesso no tópico /initialpose. Encerrando o nó.')
        
        # Opcional: Para encerrar o nó após a publicação, o que é comum para o initialpose
        self.timer_.cancel() # Cancela o timer
        rclpy.shutdown()     # Encerra o rclpy

def main(args=None):
    rclpy.init(args=args)
    
    initial_pose_publisher = InitialPosePublisher()
    
    # Usa o spin para manter o nó ativo e permitir que o timer execute a função publish_initial_pose_once
    rclpy.spin(initial_pose_publisher) 

if __name__ == '__main__':
    main()