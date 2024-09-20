import re
#import packaging
import matplotlib.pyplot as plt
import numpy as np
import random
# ---------------------
# ------Functions------
def clctData (traceLength, traceIndex, listMatrix, listIndex):
	global content
	for i in range(traceLength): 
		temp_i					= traceIndex[i]
		temp_log				= re.findall(r'\d+', content[temp_i])
		msg						= int(temp_log[-1])
		if  msg >= shake and msg < traceLength:
			listMatrix[msg - shake, 0]      = msg
			if listIndex==1:    #start
				msg_pub             = int(temp_log[-4])
				listMatrix[msg - shake, 1]  = msg_pub
			elif listIndex==2:  #end
				msg_recv            = int(temp_log[-3])
				listMatrix[msg - shake, 2]  = msg_recv

global unit
unit = 2
global shake
shake = 100

# MANY_CHAINS
chain1_start    = "Chain1-Sensor"
chain1_end      = "Chain1-Command"
chain2_start    = "Chain2-Sensor"
chain2_end      = "Chain2-Command"

file1  = "/home/neu/Desktop/OpenMP/results/Test5/DEFAULT.log"
file2  = "/home/neu/Desktop/OpenMP/results/Test5/OPENMP.log"

# --->>> 读取数据 ROS2 --->>>
with open(file1, 'r', encoding='utf-8') as f:
		content = f.readlines()
# ------>>------>>------>>
idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
size_c1         = min(len(idx_c1Start), len(idx_c1End))
mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
clctData(size_c1, idx_c1Start, mtrx_c1, 1)
clctData(size_c1, idx_c1End, mtrx_c1, 2)
if unit == 1 :
	temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000
	chain1_ltcy     = temp[temp > 0]
else :
	temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
	chain1_ltcy     = temp[temp > 0]
# ------>>------>>------>>
idx_c2Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
idx_c2End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
size_c2         = min(len(idx_c2Start), len(idx_c2End))
mtrx_c2         = np.zeros([size_c2 - shake, 3], dtype = int)
clctData(size_c2, idx_c2Start, mtrx_c2, 1)
clctData(size_c2, idx_c2End, mtrx_c2, 2)
if unit == 1 :
	temp = (mtrx_c2[:,2]-mtrx_c2[:,1])/1000
	chain2_ltcy     = temp[temp > 0]
else :
	temp = (mtrx_c2[:,2]-mtrx_c2[:,1])/1000000
	chain2_ltcy     = temp[temp > 0]

default1 = chain1_ltcy
default2 = chain2_ltcy


with open(file2, 'r', encoding='utf-8') as f:
		content = f.readlines()
# ------>>------>>------>>
idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
size_c1         = min(len(idx_c1Start), len(idx_c1End))
mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
clctData(size_c1, idx_c1Start, mtrx_c1, 1)
clctData(size_c1, idx_c1End, mtrx_c1, 2)
if unit == 1 :
	temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000
	chain1_ltcy     = temp[temp > 0]
else :
	temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
	chain1_ltcy     = temp[temp > 0]
# ------>>------>>------>>
idx_c2Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
idx_c2End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
size_c2         = min(len(idx_c2Start), len(idx_c2End))
mtrx_c2         = np.zeros([size_c2 - shake, 3], dtype = int)
clctData(size_c2, idx_c2Start, mtrx_c2, 1)
clctData(size_c2, idx_c2End, mtrx_c2, 2)
if unit == 1 :
	temp = (mtrx_c2[:,2]-mtrx_c2[:,1])/1000
	chain2_ltcy     = temp[temp > 0]
else :
	temp = (mtrx_c2[:,2]-mtrx_c2[:,1])/1000000
	chain2_ltcy     = temp[temp > 0]

openmp1 = chain1_ltcy
openmp2 = chain2_ltcy

print(np.average(default1))
print(np.average(default2))
print(np.average(openmp1))
print(np.average(openmp2))
# print(np.average(default1))
# print(np.average(default2))
# print(np.average(openmp1))
# print(np.average(openmp2))

chain1 = np.asarray([default1, openmp1], dtype=object)  
chain2 = np.asarray([default2, openmp2], dtype=object)

labels      = (["default", "openmp"])
colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(0,0.5),widths=0.2)
for patch, color in zip(bplot_1['boxes'], colors):
				patch.set_facecolor(color)
				patch.set(linewidth=0.5)
bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
for patch, color in zip(bplot_2['boxes'], colors):
				patch.set_facecolor(color)
				patch.set(linewidth=0.5)

x_position  = [0, 1]
x_posn_fmt  = ["Chain1", "Chain2"]

plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
plt.yticks(fontsize=10)
if unit == 1 :
	plt.ylabel('Latency [$\mu$s]', fontsize=13)
else :
    plt.ylabel('Latency [$m$s]', fontsize=13)
plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
plt.legend(bplot_2['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
#plt.savefig(fname="pic.png",figsize=[10,10])  
plt.show()