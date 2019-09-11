# coding:utf-8
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
walls=np.array([[1,1]])
# agentの設定
agents = []
# desiredV,actualV,pos,dest
agent=Agent(0.1,np.array([0, 0]),np.array([0, 0]), np.array([10.0, 10.0]))
agents.append(agent)
agent=Agent(0.1,np.array([0, 0]),np.array([0, 10]), np.array([10.0, 0.0]))
agents.append(agent)

# 描画用
fig = plt.figure()
ims = []

# 相互作用を計算
for i in range(step):
    positions = [[0 for i in range(2)] for j in enumerate(agents)]
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
            #import pdb; pdb.set_trace()
            wallInter += ai.wallInteraction(wall)

        # 合力
        sumForce = forcetogoal + peopleInter  #+ wallInter
        #print(sumForce)
        accl=sumForce/ai.mass #加速度の計算
        ai.actualV=v0+accl*dt #速度
        ai.pos = r0 + v0 * dt + 0.5 * accl * dt * dt  #位置
        #positions.append([ai.pos)
        positions[0][idxi]=ai.pos[0]
        positions[1][idxi] = ai.pos[1]
        #import pdb; pdb.set_trace()
        #print("Agent"+str(idxi)+"位置="+str(ai.pos))
    fig = plt.figure()
    plt.plot(positions[0],positions[1],"-o", linestyle='None', color='black')
    plt.plot(walls)
    ims.append(fig)
ani = animation.ArtistAnimation(fig, ims, interval=200)
plt.show()