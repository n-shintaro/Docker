#!/usr/bin/python
# coding:utf-8
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from agent import *
from config import *
from geometry_msgs.msg import Point, Twist
#import pdb; pdb.set_trace()
# global変数
AGENTSNUM=5
dt = 0.5
T = 200
step=int(200/dt)
# 壁の設定
walls=np.array([[3, 8],[8, 8],[4, 3]])
# agentの設定
agents = []
# desiredV,actualV,pos,dest
agent1=Agent(0.1,np.array([0, 0]),np.array([0, 0]), np.array([10.0, 10.0]))
agents.append(agent1)
agent2=Agent(0.1,np.array([0, 0]),np.array([0, 10]), np.array([10.0, 0.0]))
agents.append(agent2)
################################################################################
#                             SFM Navigation                                 #
################################################################################
class SFMNavigation:
    #===========================================================================
    #   Constructor
    #===========================================================================
    def __init__(self):
        print "\n============== SFM Navigation ==============="

        # Initialize node ------------------------------------------------------
        rospy.init_node("sfm_navigation")
    # 速度指令 Publisher ----------------------------------------------------
        self.position_publisher = rospy.Publisher('pos_vel', Point, queue_size=1)
    #===========================================================================
    #   Calculate SFM
    #===========================================================================

    def social_force_model(self,agents, walls):
        cml_pos =Point()
        # 相互作用を計算
        for i in range(step):
            r = rospy.Rate(20)     # 20Hz
            for idxi,ai in enumerate(agents):# idxi: インデックス　ai：要素

                # 初期速度と位置
                v0 = ai.actualV
                r0 = ai.pos
                ai.direction = normalize(ai.dest - ai.pos)

                #　ゴールまでへの力
                forcetogoal=ai.adaptVel()

                # 人との相互作用
                peopleInter=0.0
                for idxj,aj in enumerate(agents):
                    if idxi == idxj:
                        continue
                    peopleInter += ai.peopleInteraction(aj)

                # 障害物からの力
                wallInter=0.0
                for idxj,wall in enumerate(walls):
                    wallInter += ai.wallInteraction(wall)
                # 合力
                sumForce = forcetogoal + peopleInter + wallInter

                accl=sumForce/ai.mass #加速度の計算
                ai.actualV=v0+accl*dt #速度
                ai.pos = r0 + v0 * dt + 0.5 * accl * dt * dt  #位置

                # 位置をpublish
                cml_pos.x = ai.pos[0]
                cml_pos.y=ai.pos[1]
                self.position_publisher.publish(cml_pos)
                r.sleep()

################################################################################
#                               Main Function                                  #
################################################################################
if __name__ == '__main__':
    robot   = SFMNavigation()
    robot.social_force_model(agents, walls)
