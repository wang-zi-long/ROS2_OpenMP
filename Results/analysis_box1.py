import re
import numpy as np
import random
import os
import matplotlib.pyplot as plt
chain1_start = 'Chain1-Sensor1'
chain1_end = 'Chain1-Command2'
chain2_start = 'Chain2-Sensor1'
chain2_end = 'Chain2-Command2'
chain3_start = 'Chain2-Sensor1'
chain3_end = 'Chain3-Command4'
chain4_start = 'Chain4-Sensor1'
chain4_end = 'Chain4-Command4'
chain5_start = 'Chain5-Sensor1'
chain5_end = 'Chain5-Command3'
chain6_start = 'Chain6-Sensor1'
chain6_end = 'Chain6-Command4'
file_path = "/home/neu/Desktop/Openmp/results/10.15/" #读取的文件夹的路径
file_pat1 = "/home/neu/Desktop/Openmp/results/FIFO/"
Picture_save_path = '/home/neu/Desktop/OMP/Results/10.29_1/1_to_8/' #图片保存的位置，这个是非箱型图保存的位置
Picture_save_path_box = '/home/neu/Desktop/Openmp/results/Picture/three/'
x = [i for i in range(1,6)]
mean = [0]*6
var = [0]*6
mean1 = [0]*6
mean_all = [0]*19
var_all = [0]*19
global shake
shake = 0

def read_and_handle(chain_start,deafult):
    with open(deafult, 'r', encoding='utf-8') as f:
        content = f.readlines()
    idx_c1Start     = [x for x in range(len(content)) if (chain_start) in content[x]]
    idx_c1Start     = idx_c1Start[300:-300]

    num_len = len(idx_c1Start)
    data = [0]*num_len
    for i in range(0,num_len):
        str_line = content[idx_c1Start[i]]
        str_line = re.findall(r'\d+', str_line)        
        data[i] = int(str_line[-2]) / 1000000
    # data = np.array(data)
    # print(np.mean(data))
    # print(np.std(data))
    return data


def os_read_box(num,chain1_ltcy_default,chain1_ltcy_openmp):
    min_length = min(len(chain1_ltcy_default), len(chain1_ltcy_openmp))
    chain1_ltcy_424 = chain1_ltcy_default[:min_length]
    chain1_ltcy_425 = chain1_ltcy_openmp[:min_length]
    chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]


  
    # colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.), (120/255.,214/255.,201/255.), (120/255.,214/255.,201/255.), (120/255.,214/255.,201/255.), (120/255.,214/255.,201/255.)]
    colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.), (120/255.,214/255.,201/255.)]
    plt.figure(figsize=(30,20))
    labels = ['424','425']
    bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.25),widths=0.2)
    for patch, color in zip(bplot_1['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    x_position  = [1, 2]
    x_posn_fmt  = [1, 2]

    plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
    plt.yticks(fontsize=10)
    plt.ylabel('Latency [$m$s]', fontsize=13)
    plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
    plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
    plt.savefig("/home/neu/Desktop/OMP/Results/11.1/box.png", dpi=300)
    
def exec_box(num):
    deafult = '/home/neu/Desktop/OMP/Results/11.2/1.log'
    temp1_1=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2_1/1.log'
    temp1_2=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2/2.log'
    temp2_1=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2_1/2.log'
    temp2_2=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2/3.log'
    temp3_1=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2_1/3.log'
    temp3_2=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2/4.log'
    temp4_1=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2_1/4.log'
    temp4_2=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2/5.log'
    temp5_1=read_and_handle("Chain1-Command2",deafult)
    deafult = '/home/neu/Desktop/OMP/Results/11.2_1/5.log'
    temp5_2=read_and_handle("Chain1-Command2",deafult)
    # deafult = '/home/neu/Desktop/OMP/Results/11.2/6.log'
    # temp6_1=read_and_handle("Chain1-Command2",deafult)
    # deafult = '/home/neu/Desktop/OMP/Results/11.2_1/6.log'
    # temp6_2=read_and_handle("Chain1-Command2",deafult)

    temp1 = [temp1_1, temp1_2]
    temp2 = [temp2_1, temp2_2]
    temp3 = [temp3_1, temp3_2]
    temp4 = [temp4_1, temp4_2]
    temp5 = [temp5_1, temp5_2]
    # temp6 = [temp6_1, temp6_2]

    colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]
    # plt.figure(figsize=(30,20))
    labels = ['ROS 2 + OMP', 'ROSOMP']
    bplot_1     = plt.boxplot(temp1, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.25),widths=0.2)
    for patch, color in zip(bplot_1['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    bplot_2     = plt.boxplot(temp2, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.25),widths=0.2)
    for patch, color in zip(bplot_2['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    bplot_3     = plt.boxplot(temp3, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.25),widths=0.2)
    for patch, color in zip(bplot_3['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    bplot_4     = plt.boxplot(temp4, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.25),widths=0.2)
    for patch, color in zip(bplot_4['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    bplot_5     = plt.boxplot(temp5, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5,5.25),widths=0.2)
    for patch, color in zip(bplot_5['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set(linewidth=0.5)
    # bplot_6     = plt.boxplot(temp6, patch_artist=True,labels=labels, showfliers=False, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(6,6.25),widths=0.2)
    # for patch, color in zip(bplot_6['boxes'], colors):
    #                 patch.set_facecolor(color)
    #                 patch.set(linewidth=0.5)
    
    x_position  = [1, 2, 3, 4, 5]
    x_posn_fmt  = [1, 2, 3, 4, 5]

    plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=10)
    plt.xlabel('OpenMP Callback Number', fontsize=10)
    plt.yticks(fontsize=10)
    plt.ylabel('Latency[$ms$]', fontsize=10)
    plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线_ 透明度0.3
    plt.legend(bplot_1['boxes'],labels,loc='upper right', fontsize=12)  #绘制表示框，右下角绘制
    plt.savefig("/home/neu/Desktop/OMP/Results/box_T.pdf", dpi=300)


# os_read(file_path,19)
# read_cpu_handle_17('/home/neu/Desktop/Openmp/results/',19)
# read_cs_handle()
# exec_box(file_pat1,file_path,18)
# read_cpu_handle('/home/neu/Desktop/OMP/Results/10.29_1/1_to_8/',5)
# read_cs_handle("/home/neu/Desktop/OMP/Results/10.29_1/1_to_8/",6)
exec_box(1)
