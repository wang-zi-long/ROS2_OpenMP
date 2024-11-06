import re
#import packaging
import matplotlib.pyplot as plt
import numpy as np
import random
import os
from scipy.stats import norm
import seaborn as sns
from scipy.stats import gaussian_kde

chain1_start    = "Chain1-Sensor"
chain1_end      = "Chain1-Command1."

chains = {}
chains1 = {}

for i in range(0, 11):
    chains[i] = []
    chains1[i] = []

i = 1
while i < 11 :
    deafult = "/home/neu/Results/Callback_num_work/{}.log".format(i)
    with open(deafult, 'r', encoding='utf-8') as f:
        content = f.readlines()
    idx_c1Start     = [x for x in range(len(content)) if (chain1_end) in content[x]]
    idx_c1Start     = idx_c1Start[500:-500]
    num_len = len(idx_c1Start)
    for j in range(0,num_len):
        str_line = content[idx_c1Start[j]]
        str_line = re.findall(r'\d+', str_line)        
        chains[i].append(int(str_line[-2]) / 1000000)
    deafult = "/home/neu/Results/Callback_num_work_new1/{}.log".format(i)
    with open(deafult, 'r', encoding='utf-8') as f:
        content = f.readlines()
    idx_c1Start     = [x for x in range(len(content)) if (chain1_end) in content[x]]
    idx_c1Start     = idx_c1Start[500:-500]
    num_len = len(idx_c1Start)
    for j in range(0,num_len):
        str_line = content[idx_c1Start[j]]
        str_line = re.findall(r'\d+', str_line)        
        chains1[i].append(int(str_line[-2]) / 1000000)
    i = i + 1

chain0 = [chains[0], chains1[0]]
chain1 = [chains[1], chains1[1]]
chain2 = [chains[2], chains1[2]]
chain3 = [chains[3], chains1[3]]
chain4 = [chains[4], chains1[4]]
chain5 = [chains[5], chains1[5]]
chain6 = [chains[6], chains1[6]]
chain7 = [chains[7], chains1[7]]
chain8 = [chains[8], chains1[8]]
chain9 = [chains[9], chains1[9]]
chain10 = [chains[10], chains1[10]]

with open('/home/neu/Results/TXT/Callback_num.log', 'w') as file:
    file.write(f"GOMP: mean  max  std\n")
    for i in range(1, 11) :
            file.write(f"{i} :  {round(np.mean(chains[i]), 2)}  {round(np.max(chains[i]), 2)}  {round(np.std(chains[i]), 2)}\n")

    file.write(f"\nROSOMP: mean  max  std\n")
    for i in range(1, 11) :
            file.write(f"{i} :  {round(np.mean(chains1[i]), 2)}  {round(np.max(chains1[i]), 2)}  {round(np.std(chains1[i]), 2)}\n")

colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]
# plt.figure(figsize=(30,20))
labels = ['GOMP', 'ROSOMP']
is_showflier = True
# bplot_0     = plt.boxplot(chain0, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(0, 0.25),widths=0.2)
# for patch, color in zip(bplot_0['boxes'], colors):
#                 patch.set_facecolor(color)
#                 patch.set(linewidth=0.5)
bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1, 1.25),widths=0.2)
for patch, color in zip(bplot_1['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2, 2.25),widths=0.2)
for patch, color in zip(bplot_2['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3, 3.25),widths=0.2)
for patch, color in zip(bplot_3['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4, 4.25),widths=0.2)
for patch, color in zip(bplot_4['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_5     = plt.boxplot(chain5, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5, 5.25),widths=0.2)
for patch, color in zip(bplot_5['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_6     = plt.boxplot(chain6, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(6, 6.25),widths=0.2)
for patch, color in zip(bplot_6['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_7     = plt.boxplot(chain7, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(7, 7.25),widths=0.2)
for patch, color in zip(bplot_7['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_8     = plt.boxplot(chain8, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(8, 8.25),widths=0.2)
for patch, color in zip(bplot_8['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_9     = plt.boxplot(chain9, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(9, 9.25),widths=0.2)
for patch, color in zip(bplot_9['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)
bplot_10     = plt.boxplot(chain10, patch_artist=True,labels=labels, showfliers=is_showflier, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(10, 10.25),widths=0.2)
for patch, color in zip(bplot_10['boxes'], colors):
                patch.set_facecolor(color)
                patch.set(linewidth=0.5)

x_position  = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
x_posn_fmt  = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=10)
plt.xlabel('OpenMP Callback Number', fontsize=10)
plt.yticks(fontsize=10)
plt.ylabel('Latency[$ms$]', fontsize=10)
plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线_ 透明度0.3
plt.legend(bplot_1['boxes'],labels,loc='upper right', fontsize=12)  #绘制表示框，右下角绘制
plt.show()

# 创建绘图
# plt.figure(figsize=(10, 6))

# i = 1
# while i < 11 :
#     kde = gaussian_kde(chains1[i])
#     x = np.linspace(min(chains1[i]), max(chains1[i]), 1000)
#     density = kde(x)
#     cdf = np.cumsum(density) * (x[1] - x[0])
#     plt.plot(x, cdf, label='OMP Callback number {0}'.format(i))
#     i += 1

# plt.xlabel('Latency[$ms$]')
# plt.ylabel('Probability')
# plt.grid(linestyle="--", alpha=0.8)
# plt.legend()
# plt.savefig("/home/neu/Results/TEST/Callback_num1.png", dpi=300)