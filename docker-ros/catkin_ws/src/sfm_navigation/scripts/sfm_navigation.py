#!/usr/bin/python
# coding:utf-8
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from agent import *
from config import *
from geometry_msgs.msg import PointStamped, Twist
#import pdb; pdb.set_trace()
# global変数
AGENTSNUM=5
dt = 0.4
T = 2000
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


    # 速度指令 Publisher ----------------------------------------------------
        self.position_publisher = rospy.Publisher('cml_pos', PointStamped, queue_size=10)
    #===========================================================================
    #   Calculate SFM
    #===========================================================================

    def social_force_model(self,agents, walls):
        pointstamped = PointStamped()
        # 相互作用を計算
        while not rospy.is_shutdown():
            r = rospy.Rate(40)     # 20Hz
            for idxi,ai in enumerate(agents):# idxi: インデックス　ai：要素

                # 初期速度と位置
                v0 = ai.actualV
                r0 = ai.pos
                ai.direction = normalize(ai.dest - ai.pos)

                #ゴールまでへの力
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
                    print(wallInter)
                # 合力
                sumForce = forcetogoal + peopleInter + wallInter

                accl=sumForce/ai.mass #加速度の計算
                ai.actualV=v0+accl*dt #速度
                ai.pos = r0 + v0 * dt + 0.5 * accl * dt * dt  #位置

                # 位置をpublish
                # 位置を指定
                pointstamped.point.x = ai.pos[0]
                pointstamped.point.y = ai.pos[1]
                # フレームを指定
                pointstamped.header.frame_id = "world"
                self.position_publisher.publish(pointstamped)
                r.sleep()

################################################################################
#                               Main Function                                  #
################################################################################
if __name__ == '__main__':
    # Initialize node ------------------------------------------------------
    rospy.init_node("sfm_navigation_node")
    robot   = SFMNavigation()
    robot.social_force_model(agents, walls)
