#!/usr/bin/python
#coding:utf-8
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from agent import *
from config import *
# global変数
AGENTSNUM=5
dt = 0.5
T = 200
step=int(200/dt)
# 壁の設定
walls=np.array([[3, 8],[8, 8],[4, 3]])

# 描画用の壁
wall_draw= [[0 for i in enumerate(walls)] for j in  range(2)]
for idxj, wall in enumerate(walls):
    #import pdb; pdb.set_trace()
    wall_draw[0][idxj]=wall[0]
    wall_draw[1][idxj] = wall[1]
# agentの設定
agents = []
# desiredV,actualV,pos,dest
agent1=Agent(0.1,np.array([0, 0]),np.array([0, 0]), np.array([10.0, 10.0]))
agents.append(agent1)
agent2=Agent(0.1,np.array([0, 0]),np.array([0, 10]), np.array([10.0, 0.0]))
agents.append(agent2)

# 描画用
fig, ax = plt.subplots(1,1,figsize=(6,4))
ims = []

# 相互作用を計算
for i in range(step):
    positions = [[0 for i in range(2)] for j in enumerate(agents)]
    destination = [[0 for i in range(2)] for j in enumerate(agents)]
    velocity = [[0 for i in range(2)] for j in enumerate(agents)]
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
        #positions.append([ai.pos)
        positions[0][idxi]=ai.pos[0]
        positions[1][idxi] = ai.pos[1]
        velocity[0][idxi] = ai.actualV[0]
        velocity[0][idxi] = ai.actualV[1]
        destination[0][idxi]=ai.dest[0]
        destination[1][idxi] = ai.dest[1]
        #print("Agent"+str(idxi)+"位置="+str(ai.pos))
    im1 = ax.plot(positions[0],positions[1],"-o", linestyle='None', color='black')
    im2 = ax.plot(walls[:, 0], walls[:, 1], "-o", linestyle='None', color='blue')
    im3 = ax.plot(destination[0], destination[1], "x", linestyle='None', color='red', ms='10')
    #im4=ax.arrow(positions[0][0],positions[0][1],velocity[0][0],velocity[0][1],width=0.01,head_width=0.05,head_length=0.2,length_includes_head=True,color='k')
    im=im1+im2+im3
    ims.append(im)
ani = animation.ArtistAnimation(fig, ims, interval=200)
plt.show()
