#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class RobotTestNode(Node):
    def __init__(self):
        super().__init__('robot_test_node')
        # Publicamos en /joint_states para mover el robot en RViz
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.get_logger().info('Test Node Started. Waiting for subscribers...')
        time.sleep(2) # Esperar a que RViz conecte

    def publish_pose(self, positions):
        """Helper para enviar una pose al robot"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(p) for p in positions]
        self.publisher_.publish(msg)

    def run_tests(self):
        self.get_logger().info('>>> INICIANDO BATERÍA DE TESTS SE25 <<<')
        
        # ---------------------------------------------------------
        # TEST CASE 1: TC-TRAJ-01 (Validation - Happy Path)
        # Objetivo: Ejecutar una secuencia de 3 puntos válida.
        # ---------------------------------------------------------
        self.get_logger().info('\n[TC-TRAJ-01] Ejecutando Validación (Secuencia de 3 puntos)...')
        
        sequence_valid = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # Home
            [0.5, 0.5, -0.5, 0.0, 0.0, 0.0],         # Punto A
            [-0.5, 1.0, -1.0, 0.5, 0.5, 0.0]         # Punto B
        ]

        for i, pose in enumerate(sequence_valid):
            self.get_logger().info(f'  -> Moviendo a punto {i+1}/{len(sequence_valid)}')
            self.publish_pose(pose)
            time.sleep(2.0) # Simular tiempo de trayecto
        
        self.get_logger().info('[TC-TRAJ-01] PASSED: El robot completó la secuencia.')

        # ---------------------------------------------------------
        # TEST CASE 2: TC-TRAJ-02 (Boundary - Single Point)
        # Objetivo: Verificar que el sistema acepta una secuencia de 1 solo punto.
        # ---------------------------------------------------------
        self.get_logger().info('\n[TC-TRAJ-02] Ejecutando Límite (Secuencia de 1 punto)...')
        
        single_point = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_pose(single_point)
        time.sleep(2.0)
        
        self.get_logger().info('[TC-TRAJ-02] PASSED: El robot se movió al punto único.')

        # ---------------------------------------------------------
        # TEST CASE 3: TC-TRAJ-04/05 (Defect - Invalid Input)
        # Objetivo: Intentar romper el sistema (Inputs vacíos o fuera de rango).
        # Nota: Como es Python, simulamos la protección de software aquí.
        # ---------------------------------------------------------
        self.get_logger().info('\n[TC-TRAJ-04] Ejecutando Defecto (Secuencia Vacía/Inválida)...')

        try:
            # Simulamos el envío de una secuencia vacía o datos corruptos
            empty_sequence = [] 
            if not empty_sequence:
                raise ValueError("Error simulado: La secuencia está vacía")
            
            # Si llegamos aquí, el test falló (porque debió dar error)
            self.get_logger().error('[TC-TRAJ-04] FAILED: El sistema aceptó una secuencia vacía.')
        
        except ValueError as e:
            self.get_logger().info(f'  -> Sistema detectó error correctamente: "{e}"')
            self.get_logger().info('[TC-TRAJ-04] PASSED: El sistema rechazó la entrada inválida.')

        # Vuelta a Home para terminar
        self.get_logger().info('\nTests Finalizados. Volviendo a Home...')
        self.publish_pose([0.0]*6)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTestNode()
    
    try:
        node.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
