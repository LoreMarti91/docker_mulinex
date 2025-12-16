from typing import List
import rclpy # serve per il nodo
from rclpy.node import Node # adesso posso usare calsse Nodo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # Importa il gruppo di callback mutuamente esclusivo
from pi3hat_moteus_int_msgs.msg import JointsCommand,JointsStates,PacketPass
from geometry_msgs.msg import PointStamped
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
import numpy as np
import os

import pandas as pd
from scipy.interpolate import interp1d
from datetime import datetime
import math
from fatigue_test.linear_traj import Trajectory
from mocap4r2_msgs.msg import RigidBodies
from pi3hat_moteus_int_msgs.msg import Counter

class JumpingTest(Node):
    def __init__(self,
                node_name = "Jumping_node", # nome del nodo
                period = 0.45e-3, ## temporizzazione
                pre_time = 5, # [s]
               
                jnt_names = ['HIP','KNEE'], # QUELLI DELLA CONFIG 
                test_name = 'prova_jump', 
                n_joint = 2, # numero di giunti
                try_jump = 1500,   # [salti da eseguire]
                try_start = 1200,
                q_0 = [0.0,0.0],# [rad]  
                q_home_2 = [-0.7,2.84],    # [rad] Posizione di homing con hip verticale per jump 1
                #q_home_1 = [2, -0.7],
                q_home_1 = [3.84,-2.84],    # [rad] Posizione di homing con hip vertical per jump 2
                #q_home_2 = [1,1],
                ratio = 9,
                delta = 10000000000000000, # [rad]
               
                l = [0.19,0.19], # [m] interasse link 1 e link 2
          
                EE_home_2 = [-0.043,-0.038], #[m] posizione cartesiana del piede in home1
                EE_step_jump_2 = [-0.043,-0.379999], #[m] posizione cartesiana del piede jump 1
                EE_landing_2 = [0.043,-0.18], #[m] posizione cartesiana del piede landing 1
                EE_home_1 = [0.043,-0.038], #[m] posizione cartesiana del piede in home 2
                EE_step_jump_1 = [0.043,-0.379999], #[m] posizione cartesiana del piede in jump 2
                EE_landing_1 = [-0.043,-0.18], #[m] posizione cartesiana del piede in landing 2
                
                # offset = [math.pi,0.0],
                offset = [0.0,0.0],
        
                
                t_pre_homing = 5,   #[s] tempo prima di inizio homing
                t_homing = 5,   # [s] Durata fase di homing
                t_holding_home = 5,   # [s] mantenimento homing prima del salto
                t_jump_1 = 0.2,   # [s] FAse iniziale di salto: coppie in feedforward (raggiungimento di q_step)
                # t_jump_1 = 5,   # [s] FAse iniziale di salto: coppie in feedforward (raggiungimento di q_step)
                t_end_jump_1 = 0.5,   # [s] Fase finale salto, post coppie feedforward ( raggiunemineto di q_step_final)
                # t_end_jump_1_b = 0.5,
                t_end_jump_1_b = 0.0,
                t_homing_2 = 2, # [s] per andare in homing 2           
                t_holding_home_2 = 1, # [s] mantenimento homing 2 prima di jump 2
                t_jump_2 = 0.2, #[s] tempo per jump 2
                # t_jump_2 = 5, #[s] tempo per jump 2
                t_end_jump_2 = 0.5,   # [s] Fase finale salto, post coppie feedforward ( raggiunemineto di q_step_final)
                # t_end_jump_2_b = 0.5,
                t_end_jump_2_b = 0.0,
                t_stay = 1.0,   #[s] Tempo di attesa in posizione homing prima di un nuovo salto
                t_come_back_home = 2, #[s] Tempo per ritornare in posizione di homing
                t_pre_recovery = 5, #[s] Tempo in cui resta nella posizione 
                t_recovery = 4, #[s] Tempo che impiega a tornare in presa zero
                t_recovery_stay = 5 #[s] Tempo che resta in zero 
                ):
                
                
        pakage_path = '/home/lorenzo/Desktop/LORENZO_WS/softlegjump_ws/src/fatigue_test/' 
        super().__init__(node_name) #super chiama il costruttore della calsse Node e dà il nome ( node_name = Jumping_node)
        self.q_0 = q_0
        # self.hip_position = hip_position
        # self.knee_position = knee_position
        # self.hip_sec_enc_pos = hip_position_sec
        # self.knee_sec_enc_pos = knee_position_sec
        self.hip_position = 0.0
        self.knee_position = 0.0
        self.hip_sec_enc_pos = 0.0
        self.knee_sec_enc_pos = 0.0
        self.q_home_1 = q_home_1
        self.q_home_2 = q_home_2     
      
  
       
        # self.q_recovery = q_recovery
        self.q_recovery = [0.0,0.0]
       

        self.l = l
        self.EE_home_1 = EE_home_1
        self.EE_step_jump_1 = EE_step_jump_1   
        self.EE_landing_1 = EE_landing_1
        self.EE_home_2 = EE_home_2
        self.EE_step_jump_2 = EE_step_jump_2
        self.EE_landing_2 = EE_landing_2

        
        self.offset = offset
        self.q_pos_IK = [0.0,0.0]

        ### TEMPI FASE HOMING_CALLBACK ( AL TERMINE AZZEO COUNTER E QUINDI ANCHE DELTA TEMPO)
        self.t_pre_homing = t_pre_homing 
        self.t_homing = self.t_pre_homing + t_homing 
        self.t_holding_home = self.t_homing + t_holding_home
        #self.t_homing = [self.t_pre_homing,self.t_homing,self.t_holding_home]
        self.t_jump_1 = t_jump_1 
        self.t_end_jump_1 = self.t_jump_1 + t_end_jump_1 
        self.t_end_jump_1_b = self.t_end_jump_1 + t_end_jump_1_b
        self.t_homing_2 = self.t_end_jump_1_b + t_homing_2 
        self.t_holding_home_2 = self.t_homing_2 + t_holding_home_2 
        self.t_jump_2 = self.t_holding_home_2 + t_jump_2 
        self.t_end_jump_2 = self.t_jump_2 + t_end_jump_2 
        self.t_end_jump_2_b = self.t_end_jump_2 + t_end_jump_2_b 
        self.t_come_back_home =  t_come_back_home + self.t_end_jump_2_b 
        self.t_stay = t_stay+ self.t_come_back_home 
        self.t_pre_recovery = t_pre_recovery
        self.t_recovery = self.t_pre_recovery + t_recovery
        self.t_recovery_stay = self.t_recovery +t_recovery_stay
        self.try_jump = try_jump 
        
        # create publisher with timer 
        self.pub = self.create_publisher(JointsCommand,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                        "joint_controller/command",
                                        10)
        
        self.pub_traj = self.create_publisher(PointStamped,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                        "EE_traj",
                                        10)
        
        self.pub_traj_sts = self.create_publisher(PointStamped,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                       "EE_traj_sts",
                                       10)

        self.pub_count = self.create_publisher(Counter,                     # il nodo pubblica messaggi di tipo JointsCommand, sul topic chiamato joint_controller/command e la dimensione della coda è 10
                                       "Counter",
                                       10)
        

        self.period = period
        self.counter_try_jump = try_start 
        self.jnt_names = jnt_names
        self.n_joint = n_joint
      
        self.clock = self.get_clock()
        
      
        self.phase_jump = 0
        self.joint_data= {}
        self.ratio = ratio
        self.delta = delta
    
        self.traj_linear_ik = Trajectory() # Creiamo un'istanza della classe Trajectory

        
        ###### Creo un subsscriber: il nodo riceve messaggi del tipo Joinstate, sul topic state_broadcaster/..  non ci sono timer perchè la callback_state   è chiamata appena riceve un messaggio;
        self.state_sub = self.create_subscription(JointsStates,
                                                "state_broadcaster/joints_state",
                                                self.callback_state,
                                                10
                                                #callback_group = self.callback_group)
                                                    )


        

        ####################################################### allocaion and set up writer to save data via ros2bag
        
       
        self.start_node = self.time_to_s(self.clock.now(), 0.0)
        # self.q_0= [self.hip_position,self.knee_position]
     
        self.EE_0 = self.traj_linear_ik.Direct_Kinematic(self.q_0,self.l,self.offset)
        self.K_quadrante = 0.0
        self.EE_homing_1_now = [0.0,0.0]
        self.EE_holding_1_now = [0.0,0.0]
        self.EE_jump_1_now = [0.0,0.0]
        self.EE_landing_1_now = [0.0,0.0]
        self.EE_landing_1_b_now = [0.0,0.0]
        self.EE_homing_2_now = [0.0,0.0]
        self.EE_holding_2_now = [0.0,0.0]
        self.EE_jump_2_now = [0.0,0.0]
        self.EE_landing_2_now = [0.0,0.0]
        self.EE_landing_2_b_now = [0.0,0.0]
        self.EE_recovery_now = [0.0,0.0]
        self.EE_recovery_stay_now = [0.0,0.0]
        self.EE_q = [0.0,0.0]
        
        # print("counter_try_jump:", self.counter_try_jump, type(self.counter_try_jump))
        # print("phase_jump:", self.phase_jump, type(self.phase_jump))
        #self.timer = self.create_timer(self.period, self.homing_callback,callback_group = self.callback_group) # si crea un timer con una callback da eseguire ogni periodo (self.period)
        self.timer = self.create_timer(self.period, self.homing_callback)
        
        ######################## la callback crea messaggi e li pubblica
 
        
    def homing_callback(self):  
        self.t = self.time_to_s(self.clock.now(),self.start_node)    
        
        ### PRE HOMING
        if(self.t < self.t_pre_homing):

                   
            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0, effort = [0.0,0.0],pos_init = self.EE_0,
                             pos_fin = self.EE_0,time_ini = 0.0,time_fin=self.t_pre_homing, k_quadrante = -1.0,phase_jump = 0)

            self.EE_homing_1_now = self.EE_q
            
           
            ### hOMING
        elif (self.t >= self.t_pre_homing) and (self.t < self.t_homing):
            
            
            
            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.8,kd_scale = 1.0, effort = [0.0,0.0], pos_init = self.EE_homing_1_now,
                             pos_fin = self.EE_home_1,time_ini = self.t_pre_homing, time_fin = self.t_homing,k_quadrante = -1.0,phase_jump = 0)
           
            self.EE_holding_1_now = self.EE_q
     
            #print(f'self.q_0 {self.q_0} and  delta is {self.delta}')
            
            #### HOLDING
        elif(self.t >= self.t_homing) and (self.t < self.t_holding_home):

            

            self.publish_msg(velocity = [0.0,0.0],kp_scale =0.8,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_holding_1_now,
                             pos_fin = self.EE_home_1,time_ini = self.t_homing,time_fin = self.t_holding_home,k_quadrante = -1.0,phase_jump = 0)
                             
            self.EE_jump_1_now = self.EE_q
                   
        else:     
            self.start_node = self.time_to_s(self.clock.now(), 0.0)   
            self.timer.destroy()
            print("[INFO]\tend homing ::::::: jump") 
                    
            self.timer = self.create_timer(self.period, self.timer_callback_iteration) ## starting multi jum  ## starting the real jump   se commento questo non va alla fase di salto
    
    def timer_callback_jump(self):
        self.t = self.time_to_s(self.clock.now(),self.start_node)
                       
        #### JUMP PHASE 1 - Position step
        if(self.t < self.t_jump_1): 
             

            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_jump_1_now, 
                             pos_fin = self.EE_step_jump_1,time_ini = 0.0,time_fin = self.t_jump_1,k_quadrante = -1.0,phase_jump = 1) 
            self.EE_landing_1_now = self.EE_q       
                       
        #     ### LANDING 1
        # elif (self.t >= self.t_jump_1) and (self.t < self.t_end_jump_1): 

        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_step_jump_1, 
        #                       pos_fin = self.EE_landing_1,time_ini = self.t_jump_1, time_fin = self.t_end_jump_1,k_quadrante = 1.0)        
        #     self.EE_landing_1_b_now = self.EE_q
        
             ### LANDING 1
        elif (self.t >= self.t_jump_1) and (self.t < self.t_end_jump_1): 
            

            self.publish_msg(velocity = [0.0,0.0],kp_scale = self.traj_linear_ik.linear_trajectory(pos_init=0.6,pos_fin = 0.01,time_ini = self.t_jump_1,time_fin = self.t_end_jump_1,time = self.t),kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_step_jump_1, 
                              pos_fin = self.EE_landing_1,time_ini = self.t_jump_1, time_fin = self.t_end_jump_1,k_quadrante = 1.0,phase_jump = 2)        
            self.EE_landing_1_b_now = self.EE_q
            

        #    ## LANDING 1 B  
        # elif (self.t >= self.t_end_jump_1) and (self.t < self.t_end_jump_1_b):
             
        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.03,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_landing_1_b_now,
        #                       pos_fin = self.EE_home_2, time_ini = self.t_end_jump_1, time_fin = self.t_homing_2,k_quadrante = 1.0)        
        #     self.EE_holding_2_now = self.EE_q
        
        #    ## HOMING 2
        # elif (self.t >= self.t_end_jump_1_b) and (self.t < self.t_homing_2): 
        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.5,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_holding_2_now,
        #                       pos_fin = self.EE_home_2, time_ini = self.t_end_jump_1, time_fin = self.t_homing_2,k_quadrante = 1.0)        
        #     self.EE_holding_2_now = self.EE_q
            
            ## HOMING 2
        elif (self.t >= self.t_end_jump_1) and (self.t < self.t_homing_2):
             
            self.publish_msg(velocity = [0.0,0.0],kp_scale = self.traj_linear_ik.linear_trajectory(pos_init=0.01,pos_fin = 0.8,time_ini = self.t_end_jump_1,time_fin = self.t_homing_2, time= self.t),kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_landing_1_b_now,
                              pos_fin = self.EE_home_2, time_ini = self.t_end_jump_1, time_fin = self.t_homing_2,k_quadrante = 1.0,phase_jump = 3)        
            self.EE_holding_2_now = self.EE_q
          
        ### HOLDING HOME 2
        elif (self.t >= self.t_homing_2) and (self.t < self.t_holding_home_2): 

            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.8,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_home_2,
                             pos_fin = self.EE_home_2,time_ini = self.t_homing_2,time_fin = self.t_holding_home_2,k_quadrante = 1.0,phase_jump = 4 )        
            self.EE_jump_2_now = self.EE_q
           
        ### JUMP 2
        elif (self.t >= self.t_holding_home_2) and (self.t < self.t_jump_2): 

            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_home_2,
                              pos_fin = self.EE_step_jump_2,time_ini = self.t_holding_home_2,time_fin = self.t_jump_2,k_quadrante = 1.0,phase_jump = 5 )        
            self.EE_landing_2_now = self.EE_q
            

        # ### LANDING 2   
        # elif (self.t >= self.t_jump_2) and (self.t < self.t_end_jump_2): 

        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_step_jump_2,
        #                      pos_fin = self.EE_landing_2,time_ini = self.t_jump_2,time_fin = self.t_end_jump_2,k_quadrante = -1.0 )        
        #     self.EE_landing_2_b_now = self.EE_q
        
         ### LANDING 2   
        elif (self.t >= self.t_jump_2) and (self.t < self.t_end_jump_2): 

            self.publish_msg(velocity = [0.0,0.0],kp_scale = self.traj_linear_ik.linear_trajectory(pos_init=0.6,pos_fin = 0.01,time_ini = self.t_jump_2,time_fin = self.t_end_jump_2,time =self.t),kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_step_jump_2,
                             pos_fin = self.EE_landing_2,time_ini = self.t_jump_2,time_fin = self.t_end_jump_2,k_quadrante = -1.0,phase_jump = 6 )        
            self.EE_landing_2_b_now = self.EE_q


        # ## LANDING 2 B  
        # elif (self.t >= self.t_end_jump_2) and (self.t < self.t_end_jump_2_b): 
        

        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.03,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_landing_2_b_now,
        #                      pos_fin = self.EE_landing_2,time_ini = self.t_jump_2,time_fin = self.t_end_jump_2,k_quadrante = -1.0 )        
        #     self.EE_homing_1_now = self.EE_q
             
          
        #     ### COME BACK HOME 1
        # elif (self.t >= self.t_end_jump_2_b) and (self.t < self.t_come_back_home): 

        #     self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.8,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_homing_1_now,
        #                       pos_fin = self.EE_home_1,time_ini = self.t_end_jump_2,time_fin = self.t_come_back_home,k_quadrante = -1.0)        
        #     self.EE_holding_1_now = self.EE_q

            ## COME BACK HOME 1

        elif (self.t >= self.t_end_jump_2) and (self.t < self.t_come_back_home):

            self.publish_msg(velocity = [0.0,0.0],kp_scale = self.traj_linear_ik.linear_trajectory(pos_init=0.01,pos_fin = 0.8,time_ini = self.t_end_jump_2,time_fin = self.t_come_back_home,time =self.t),kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_landing_2_b_now,
                              pos_fin = self.EE_home_1,time_ini = self.t_end_jump_2,time_fin = self.t_come_back_home,k_quadrante = -1.0,phase_jump = 7)        
            self.EE_holding_1_now = self.EE_q
            
            
            
            
        
        elif (self.t >= self.t_come_back_home) and (self.t < self.t_stay):

            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.8,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_home_1,
                             pos_fin = self.EE_home_1,time_ini = self.t_come_back_home,time_fin = self.t_stay,k_quadrante = -1.0,phase_jump = 7)   

            self.EE_jump_1_now = self.EE_q     
    
                   

        else:
            
          
            self.counter_try_jump = self.counter_try_jump + 1
            self.start_node = self.time_to_s(self.clock.now(), 0.0)            
            print(self.counter_try_jump)
            print("[INFO]\tend ::::::: jump")

    def timer_callback_iteration(self):
        #print(f'hip  {abs(self.hip_position - self.hip_sec_enc_pos)} and  delta is {self.delta}')
        #print(f'knee  {abs(self.knee_position - self.knee_sec_enc_pos)} and  delta is {self.delta}')
               
        if(self.counter_try_jump <= self.try_jump and abs(self.hip_position - self.hip_sec_enc_pos) < self.delta and abs(self.knee_position - self.knee_sec_enc_pos) < self.delta):
            self.timer_callback_jump()

        elif(self.counter_try_jump <= self.try_jump and abs(self.hip_position - self.hip_sec_enc_pos) >= self.delta ):
            # print(f'hip  {abs(self.hip_position - self.hip_sec_enc_pos)} and  delta is {self.delta}')
            # self.timer.destroy()
            # print('[INFO]:\tfailed jump ::::::: hip')
            # raise Exception() # 
            print(f'hip  {abs(self.hip_position - self.hip_sec_enc_pos)} and  delta is {self.delta}')
            self.start_node = self.time_to_s(self.clock.now(), 0.0)   
            self.timer.destroy()
            print('[INFO]:\tfailed jump ::::::: hip') 
            self.q_recovery = [self.hip_position,self.knee_position] 
            self.EE_recovery = traj_linear_ik.Direct_Kinematic(self.q_recovery,self.l,self.offset) 
            self.timer = self.create_timer(self.period, self.recovery_callback) ## starting recovery
  
              



        elif(self.counter_try_jump <= self.try_jump and abs(self.knee_position - self.knee_sec_enc_pos) >= self.delta ):
            # print(f'knee  {abs(self.knee_position - self.knee_sec_enc_pos)} and  delta is {self.delta}')
            # self.timer.destroy()
            # print('[INFO]:\tfailed jump ::::::: knee')
            # raise Exception() # 
            print(f'knee  {abs(self.knee_position - self.knee_sec_enc_pos)} and  delta is {self.delta}')
            self.start_node = self.time_to_s(self.clock.now(), 0.0)   
            self.timer.destroy()
            print('[INFO]:\tfailed jump ::::::: knee')  
            self.q_recovery = [self.hip_position,self.knee_position] 
            self.EE_recovery = traj_linear_ik.Direct_Kinematic(q = self.q_recovery,l = self.l,offset = self.offset)  
            
        

            self.timer = self.create_timer(self.period, self.recovery_callback) ## starting recovery
        else:
            self.timer.destroy()
            print('[INFO]:\tend jump ::::::: die')
            raise Exception() # 
        

    def recovery_callback(self):
        self.t = self.time_to_s(self.clock.now(),self.start_node)
       
            
        #### Pre Recovery
        if(self.t < self.t_pre_recovery):  

            self.EE_recovery_now = self.EE_q
           
            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.3,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_recovery,
                              pos_fin = self.EE_recovery, time_fin = self.t_pre_recovery, k_quadrante = self.traj_linear_ik.determina_quadrante(EE_pos_now = self.EE_recovery_now),phase_jump = 9  )   

                
                           
                       
            ### RECOVERY
        elif (self.t >= self.t_pre_recovery) and (self.t < self.t_recovery): 
            self.EE_recovery_stay_now = self.EE_q 

            self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_recovery_now,
                             pos_fin = self.EE_0,time_fin = self.t_recovery,k_quadrante =self.traj_linear_ik.determina_quadrante(EE_pos_now = self.EE_recovery_stay_now),phase_jump = 10)

                 
             
   
           
            
           ## STAY RECOVERY
        elif (self.t >= self.t_recovery) and (self.t < self.t_recovery_stay): 

             self.publish_msg(velocity = [0.0,0.0],kp_scale = 0.6,kd_scale = 1.0,effort = [0.0,0.0],pos_init = self.EE_recovery_stay_now,
                              pos_fin = self.EE_recovery_stay_now,time_fin = self.t_recovery_stay, k_quadrante =self.traj_linear_ik.determina_quadrante(EE_pos_now = self.EE_recovery_stay_now),phase_jump = 11)        
            
        

        else:
            self.timer.destroy()
            print("[INFO]\tend ::::::: recovery")
            raise Exception() #                 
                    
             
  
    def publish_msg(self,velocity,kp_scale,kd_scale,effort,pos_init,pos_fin,time_ini,time_fin,k_quadrante,phase_jump): 
        msg = JointsCommand()
        msg_count = Counter()
        self.phase_jump = phase_jump   
        self.q = [self.hip_position,self.knee_position]
        #self.q = [0.0,0.0]
        # print('self.q', self.q)
        self.EE_q = self.traj_linear_ik.Direct_Kinematic(q = self.q,l=self.l, offset = self.offset)
    
        

        #print('self.EE_init',self.EE_init)
        self.k_quadrante = k_quadrante
        # print('k_quadrante = ', self.k_quadrante)
        self.q_pos_IK = self.traj_linear_ik.Trajectory_parabolic_IK(pos_init = pos_init,pos_fin = pos_fin, time_ini = time_ini,time_fin = time_fin,time = self.t,l =self.l,offset = self.offset,k_quadrante = self.k_quadrante)   
        # print('self.q_pos_IK',self.q_pos_IK)
        
        if not any(math.isnan(x) for x in self.q_pos_IK):
          

        
        #self.q_dot_IK = self.traj_linear_ik.calcolo_q_dot(pos = self.q_pos_IK, dt = self.period)
            for j in range(len(self.jnt_names)):
                    msg.header.stamp = self.clock.now().to_msg()
                    msg.name.append(self.jnt_names[j])  
                    msg.position.append(self.q_pos_IK[j])
                    msg.velocity.append(velocity[j])
                    msg.kp_scale.append(kp_scale)
                    msg.kd_scale.append(kd_scale)
                    msg.effort.append(effort[j])

                    
    
            self.pub.publish(msg)

        else:
            print('C è un Nan PD')
        msg_EE = PointStamped()
        self.EE_traj = self.traj_linear_ik.Trajectory_parabolic(pos_init = pos_init,pos_fin = pos_fin ,time_ini = time_ini,time_fin = time_fin,time = self.t)
        self.EE_traj_sts = self.traj_linear_ik.Direct_Kinematic(q=self.q,l=self.l,offset=self.offset)
        # print('EE_trajPD = ',self.EE_traj)
        msg_EE.header.stamp = self.clock.now().to_msg()
        msg_EE.point.x= self.EE_traj[0]
        msg_EE.point.y = self.EE_traj[1]
        msg_EE_sts = PointStamped()
        msg_EE_sts.header.stamp = self.clock.now().to_msg()
        msg_EE_sts.point.x= self.EE_traj_sts[0]
        msg_EE_sts.point.y = self.EE_traj_sts[1]
        # print('EE_trajPDmsg = ',msg_EE)
        msg_count.header.stamp = self.clock.now().to_msg()
        msg_count.counter_jump = [self.counter_try_jump]
        msg_count.phase_jump = [self.phase_jump]
        
        self.pub_traj.publish(msg_EE)
        
        self.pub_traj_sts.publish(msg_EE_sts)

        
        self.pub_count.publish(msg_count)


        

    def time_to_s(self, time, start):
        [sec, ns] = time.seconds_nanoseconds()
        now = float(sec + ns/pow(10, 9))
        return (now - start)
  
 

    def callback_state(self, msg):


         

         for i, joint_name in enumerate(msg.name):
             position = msg.position[i]
             sec_enc_pos = msg.sec_enc_pos[i] if i < len(msg.sec_enc_pos) else None

             # Controllo per position
             if math.isnan(position):
                 #print(f"Attenzione: Valore NaN per la posizione del giunto {joint_name}")
                 # Non aggiornare il campo se è NaN
                 if joint_name in self.joint_data:
                     position = self.joint_data[joint_name].get("position")

             # Controllo per second encoder position
             if sec_enc_pos is not None and math.isnan(sec_enc_pos):
                 #print(f"Attenzione: Valore NaN per la posizione del secondo encoder del giunto {joint_name}")
                 # Non aggiornare il campo se è NaN
                 if joint_name in self.joint_data:
                     sec_enc_pos = self.joint_data[joint_name].get("sec_enc_pos")

             # Aggiorna il dizionario solo se i valori non sono NaN
             if not math.isnan(position):
                 if joint_name not in self.joint_data:
                     self.joint_data[joint_name] = {}
                 self.joint_data[joint_name]["position"] = position

             if sec_enc_pos is not None and not math.isnan(sec_enc_pos):
                 if joint_name not in self.joint_data:
                     self.joint_data[joint_name] = {}
                 self.joint_data[joint_name]["sec_enc_pos"] = sec_enc_pos

             #print(self.joint_data)

         # Gestione specifica di KNEE e HIP
         if "KNEE" in self.joint_data:
             self.knee_position = self.joint_data["KNEE"]["position"]
             self.knee_sec_enc_pos = self.joint_data["KNEE"]["sec_enc_pos"]

         if "HIP" in self.joint_data:
             self.hip_position = self.joint_data["HIP"]["position"]
             self.hip_sec_enc_pos = self.joint_data["HIP"]["sec_enc_pos"]
         




#######################################################################################################
#######################################################################################################

## ToDo: save node_name using param from launch file 


########### DEFINIZIONE DELLA FUNZIONE main
def main(args=None):
    rclpy.init(args=args)  # Si inizializza la libreria rlcpy
    name_exp = "softleg_jump_" + datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    node = JumpingTest( test_name=name_exp,q_home_1 = [-0.7,2.84],period=1/450)    # VIENE CREATO IL NODO
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    
    try:
        ex.spin()   #esegue in loop quello che è nel nodo
        print("pass here")
    except Exception as e:  # Gestisce qualsiasi eccezione
        print(f"An error occurred: {e}") # quando viene chiamata esegue questo
        print("Closing ROS2 Node")
        ex.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ =="__main__":
    main()
            
    
