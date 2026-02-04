#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
import time

class Fra2moMission(Node):
    def __init__(self):
        super().__init__('fra2mo_mission_node')
        
        # Sim Time
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.iiwa_trigger = self.create_publisher(Empty, '/iiwa/start_manipulation', 10)

        # Publisher per dire al pick_ancd_place node di fare Spawn/Delete del pacco
        self.place_trigger = self.create_publisher(Empty, '/fra2mo/trigger_place', 10) 
        
        # Sottoscrizione al segnale di fine manipolazione
        self.manipulation_done_sub = self.create_subscription(
            Empty, '/iiwa/manipulation_done', self.done_cb, 10)
        self.manipulation_completed = False

        self.aruco_pose = None
        self.last_aruco_time = 0.0 
        
        self.aruco_sub = self.create_subscription(
            PoseStamped, '/aruco_single/pose', self.aruco_cb, 10)

    def aruco_cb(self, msg):
        self.aruco_pose = msg
        self.last_aruco_time = time.time()

    def done_cb(self, msg):
        print(">>> SEGNALE RICEVUTO: IIWA HA FINITO! PACCO PRESO. <<<")
        self.manipulation_completed = True

    def visual_servoing(self):
        print(">>> INIZIO VISUAL SERVOING <<<")
        twist = Twist()
        
        target_distance = 0.80 
        x_tolerance = 0.06     
        y_tolerance = 0.15 
        
        start_time = time.time()
        
        # Variabili Anti-Blocco
        last_x_error = 0.0
        stuck_timer = 0
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if (time.time() - start_time) > 60.0:
                print("⚠️ TIMEOUT VISUAL SERVOING. Procedo.")
                self.cmd_vel_pub.publish(Twist())
                return True
            #WATCHDOG TIMER
            if self.aruco_pose is None or (time.time() - self.last_aruco_time > 1.0):
                continue

            x_error = self.aruco_pose.pose.position.x - target_distance
            y_error = self.aruco_pose.pose.position.y 
            
            # CHECK TARGET
            if abs(x_error) < x_tolerance and abs(y_error) < y_tolerance:
                print(f"✅ POSIZIONE RAGGIUNTA! (ErrX: {x_error:.3f}, ErrY: {y_error:.3f})")
                self.cmd_vel_pub.publish(Twist()) 
                return True
            
            # STUCK DETECTOR
            if abs(x_error - last_x_error) < 0.001: stuck_timer += 1
            else: stuck_timer = 0
            last_x_error = x_error

            if stuck_timer > 30:
                print(f"⚠️ STALLO RILEVATO. Accetto la posizione.")
                self.cmd_vel_pub.publish(Twist()) 
                return True

            # MOVIMENTO
            if abs(y_error) > y_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = 2.0 * y_error
                # Clamp e Deadband
                twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)
                if abs(twist.angular.z) < 0.25: twist.angular.z = 0.25 if twist.angular.z > 0 else -0.25
                
                print(f"🔄 ALLINEO... ErrY: {y_error:.3f}")
            else:
                twist.linear.x = 0.5 * x_error
                twist.angular.z = 0.8 * y_error 
                twist.linear.x = max(min(twist.linear.x, 0.25), -0.25)
                # Boost Retromarcia
                min_speed = 0.10
                if abs(twist.linear.x) < min_speed: twist.linear.x = min_speed if twist.linear.x > 0 else -min_speed
                
                print(f"⬆️ AVVICINO... ErrX: {x_error:.3f}")

            self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = Fra2moMission()
    navigator = BasicNavigator()

    print("⏳ Attendo Nav2...")
    navigator.waitUntilNav2Active()
    
    # ---------------- GOAL 1: PICKING ZONE ----------------
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.8788
    goal_pose.pose.position.y = 1.4898
    goal_pose.pose.orientation.z = 1.0 
    goal_pose.pose.orientation.w = 0.0
    
    print("🚀 Navigazione verso SCAFFALE L3...")
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete(): pass 

    if navigator.getResult() == TaskResult.SUCCEEDED:
        print("✅ Arrivato in zona Picking!")
        node.visual_servoing() # Allineamento preciso
        
        # Rotazione 180°
        print("🔄 ESEGUO SPIN DI 180 GRADI...")
        navigator.spin(spin_dist=3.14, time_allowance=10)
        while not navigator.isTaskComplete(): pass
        
        # Trigger IIWA
        print("🔔 START IIWA!")
        node.iiwa_trigger.publish(Empty())
        
        # --- WAIT FOR COMPLETION ---
        print("⏳ Attendo che IIWA finisca (ascolto topic)...")
        # Loop finché il booleano non diventa True
        while not node.manipulation_completed and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        
        print("✅ PACCO CARICATO! Inizio Fase 2...")

        # ---------------- GOAL 2: SCAFFALE R2 ----------------
        goal_pose_shelf = PoseStamped()
        goal_pose_shelf.header.frame_id = 'map'
        goal_pose_shelf.header.stamp = navigator.get_clock().now().to_msg()
        
        # Coordinate "Safe Approach"
        goal_pose_shelf.pose.position.x = 4.581
        goal_pose_shelf.pose.position.y = -1.50
        goal_pose_shelf.pose.position.z = 0.000
        
        # Orientamento forzato a -90 gradi
        goal_pose_shelf.pose.orientation.x = 0.000
        goal_pose_shelf.pose.orientation.y = 0.000
        goal_pose_shelf.pose.orientation.z = -0.707
        goal_pose_shelf.pose.orientation.w = 0.707

        print("🚀 Navigazione verso SCAFFALE R2...")
        navigator.goToPose(goal_pose_shelf)
        while not navigator.isTaskComplete(): pass
        
        if navigator.getResult() == TaskResult.SUCCEEDED:
            print("✅ Arrivato allo Scaffale R2!")
            
            # 1. Visual Servoing Finale
            print("👁️ Visual Servoing Finale...")
            node.visual_servoing()

            # 2. Piccola attesa per stabilizzare
            time.sleep(2.0)
            
            # --- INVIO SEGNALE DI SCARICO ---
            print("🎁 Allineamento OK! Invio comando SCARICA al nodo C++...")
            node.place_trigger.publish(Empty())
            
            # Aspettiamo un attimo per vedere l'effetto scenico
            time.sleep(3.0)

            print("🏁 MISSIONE COMPLETATA CON SUCCESSO! 🏁")
        else:
             print("❌ Navigazione verso Scaffale R2 fallita!")

    else:
        print("❌ Navigazione verso Picking fallita!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()