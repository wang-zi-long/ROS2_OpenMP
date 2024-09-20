import re
#import packaging
import matplotlib.pyplot as plt
import numpy as np
import random
import os
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

global shake
shake = 100

global ltcy_path
lcty_path = "/home/neu/Desktop/OpenMP/results/Random5/ltcy/"

# MANY_CHAINS
chain1_start    = "Chain1-Sensor"
chain1_end      = "Chain1-Command"
chain2_start    = "Chain2-Sensor"
chain2_end      = "Chain2-Command"
chain3_start    = "Chain3-Sensor"
chain3_end      = "Chain3-Command"
chain4_start    = "Chain4-Sensor"
chain4_end      = "Chain4-Command"
chain5_start    = "Chain5-Sensor"
chain5_end      = "Chain5-Command"
chain6_start    = "Chain6-Sensor"
chain6_end      = "Chain6-Command"
chain7_start    = "Chain7-Sensor"
chain7_end      = "Chain7-Command"
chain8_start    = "Chain8-Sensor"
chain8_end      = "Chain8-Command"

folder_path = "/home/neu/Desktop/OpenMP/results/Random5"

# 使用 os.walk() 遍历文件夹
for root, dirs, files in os.walk(folder_path):
	for dir_name in dirs:
		dir_path = os.path.join(root, dir_name)
		print(f"Path: {dir_path}")
		chain_num = dir_path.split('/')[-1].split('_')[0]
		print(f"chain_num: {chain_num}")
		last_part = dir_path.split('/')[-1]

		if chain_num == "2" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)  

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)

			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)

			x_position  = [1, 2]
			x_posn_fmt  = ["Chain1", "Chain2"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()
		elif chain_num == "3" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)  

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)

			x_position  = [1, 2, 3]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()
		
		elif chain_num == "4" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)
			if len(chain4_ltcy_default) == len(chain4_ltcy_openmp) :
				chain4 = [chain4_ltcy_default, chain4_ltcy_openmp]
			else :
				chain4 = np.asarray([chain4_ltcy_default, chain4_ltcy_openmp], dtype=object)

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.5),widths=0.2)
			for patch, color in zip(bplot_4['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)

			x_position  = [1, 2, 3, 4]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3", "Chain4"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()

		elif chain_num == "5" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_openmp     = temp[temp > 0]
			
			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)
			if len(chain4_ltcy_default) == len(chain4_ltcy_openmp) :
				chain4 = [chain4_ltcy_default, chain4_ltcy_openmp]
			else :
				chain4 = np.asarray([chain4_ltcy_default, chain4_ltcy_openmp], dtype=object)
			if len(chain5_ltcy_default) == len(chain5_ltcy_openmp) :
				chain5 = [chain5_ltcy_default, chain5_ltcy_openmp]
			else :
				chain5 = np.asarray([chain5_ltcy_default, chain5_ltcy_openmp], dtype=object)

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.5),widths=0.2)
			for patch, color in zip(bplot_4['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_5     = plt.boxplot(chain5, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5,5.5),widths=0.2)
			for patch, color in zip(bplot_5['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)


			x_position  = [1, 2, 3, 4, 5]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3", "Chain4", "Chain5"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()
		
		elif chain_num == "6" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)
			if len(chain4_ltcy_default) == len(chain4_ltcy_openmp) :
				chain4 = [chain4_ltcy_default, chain4_ltcy_openmp]
			else :
				chain4 = np.asarray([chain4_ltcy_default, chain4_ltcy_openmp], dtype=object)
			if len(chain5_ltcy_default) == len(chain5_ltcy_openmp) :
				chain5 = [chain5_ltcy_default, chain5_ltcy_openmp]
			else :
				chain5 = np.asarray([chain5_ltcy_default, chain5_ltcy_openmp], dtype=object)
			if len(chain6_ltcy_default) == len(chain6_ltcy_openmp) :
				chain6 = [chain6_ltcy_default, chain6_ltcy_openmp]
			else :
				chain6 = np.asarray([chain6_ltcy_default, chain6_ltcy_openmp], dtype=object)

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.5),widths=0.2)
			for patch, color in zip(bplot_4['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_5     = plt.boxplot(chain5, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5,5.5),widths=0.2)
			for patch, color in zip(bplot_5['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_6     = plt.boxplot(chain6, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(6,6.5),widths=0.2)
			for patch, color in zip(bplot_6['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)


			x_position  = [1, 2, 3, 4, 5, 6]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3", "Chain4", "Chain5", "Chain6"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()

		elif chain_num == "7" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain7_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain7_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain7_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain7_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain7_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain7_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)
			if len(chain4_ltcy_default) == len(chain4_ltcy_openmp) :
				chain4 = [chain4_ltcy_default, chain4_ltcy_openmp]
			else :
				chain4 = np.asarray([chain4_ltcy_default, chain4_ltcy_openmp], dtype=object)
			if len(chain5_ltcy_default) == len(chain5_ltcy_openmp) :
				chain5 = [chain5_ltcy_default, chain5_ltcy_openmp]
			else :
				chain5 = np.asarray([chain5_ltcy_default, chain5_ltcy_openmp], dtype=object)
			if len(chain6_ltcy_default) == len(chain6_ltcy_openmp) :
				chain6 = [chain6_ltcy_default, chain6_ltcy_openmp]
			else :
				chain6 = np.asarray([chain6_ltcy_default, chain6_ltcy_openmp], dtype=object)
			if len(chain7_ltcy_default) == len(chain7_ltcy_openmp) :
				chain7 = [chain7_ltcy_default, chain7_ltcy_openmp]
			else :
				chain7 = np.asarray([chain7_ltcy_default, chain7_ltcy_openmp], dtype=object)

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.5),widths=0.2)
			for patch, color in zip(bplot_4['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_5     = plt.boxplot(chain5, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5,5.5),widths=0.2)
			for patch, color in zip(bplot_5['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_6     = plt.boxplot(chain6, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(6,6.5),widths=0.2)
			for patch, color in zip(bplot_6['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_7     = plt.boxplot(chain7, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(7,7.5),widths=0.2)
			for patch, color in zip(bplot_7['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)


			x_position  = [1, 2, 3, 4, 5, 6, 7]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3", "Chain4", "Chain5", "Chain6", "Chain7"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')  
			plt.show()

		elif chain_num == "8" :
			deafult = dir_path + "/0.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain7_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain7_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain7_ltcy_default     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain8_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain8_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain8_ltcy_default     = temp[temp > 0]

			deafult = dir_path + "/1.log"

			with open(deafult, 'r', encoding='utf-8') as f:
					content = f.readlines()
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain1_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain1_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain1_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain2_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain2_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain2_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain3_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain3_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain3_ltcy_openmp     = temp[temp > 0]
			# ------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain4_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain4_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain4_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain5_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain5_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain5_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain6_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain6_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain6_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain7_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain7_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain7_ltcy_openmp     = temp[temp > 0]
			#------>>------>>------>>
			idx_c1Start     = [x for x in range(len(content)) if (chain8_start) in content[x]]
			idx_c1End       = [x for x in range(len(content)) if (chain8_end) in content[x]]
			size_c1         = min(len(idx_c1Start), len(idx_c1End))
			if size_c1 - shake < 0 :
				continue
			mtrx_c1         = np.zeros([size_c1 - shake, 3], dtype = int)
			clctData(size_c1, idx_c1Start, mtrx_c1, 1)
			clctData(size_c1, idx_c1End, mtrx_c1, 2)
			temp = (mtrx_c1[:,2]-mtrx_c1[:,1])/1000000
			chain8_ltcy_openmp     = temp[temp > 0]

			if len(chain1_ltcy_default) == len(chain1_ltcy_openmp) :
				chain1 = [chain1_ltcy_default, chain1_ltcy_openmp]
			else :
				chain1 = np.asarray([chain1_ltcy_default, chain1_ltcy_openmp], dtype=object)
			if len(chain2_ltcy_default) == len(chain2_ltcy_openmp) :
				chain2 = [chain2_ltcy_default, chain2_ltcy_openmp]
			else :
				chain2 = np.asarray([chain2_ltcy_default, chain2_ltcy_openmp], dtype=object)
			if len(chain3_ltcy_default) == len(chain3_ltcy_openmp) :
				chain3 = [chain3_ltcy_default, chain3_ltcy_openmp]
			else :
				chain3 = np.asarray([chain3_ltcy_default, chain3_ltcy_openmp], dtype=object)
			if len(chain4_ltcy_default) == len(chain4_ltcy_openmp) :
				chain4 = [chain4_ltcy_default, chain4_ltcy_openmp]
			else :
				chain4 = np.asarray([chain4_ltcy_default, chain4_ltcy_openmp], dtype=object)
			if len(chain5_ltcy_default) == len(chain5_ltcy_openmp) :
				chain5 = [chain5_ltcy_default, chain5_ltcy_openmp]
			else :
				chain5 = np.asarray([chain5_ltcy_default, chain5_ltcy_openmp], dtype=object)
			if len(chain6_ltcy_default) == len(chain6_ltcy_openmp) :
				chain6 = [chain6_ltcy_default, chain6_ltcy_openmp]
			else :
				chain6 = np.asarray([chain6_ltcy_default, chain6_ltcy_openmp], dtype=object)
			if len(chain7_ltcy_default) == len(chain7_ltcy_openmp) :
				chain7 = [chain7_ltcy_default, chain7_ltcy_openmp]
			else :
				chain7 = np.asarray([chain7_ltcy_default, chain7_ltcy_openmp], dtype=object)
			if len(chain8_ltcy_default) == len(chain8_ltcy_openmp) :
				chain8 = [chain8_ltcy_default, chain8_ltcy_openmp]
			else :
				chain8 = np.asarray([chain8_ltcy_default, chain8_ltcy_openmp], dtype=object)

			# print(np.average(chain1_ltcy_default))
			# print(np.average(chain2_ltcy_default))
			# print(np.average(chain3_ltcy_default))
			# print(np.average(chain4_ltcy_default))
			# print(np.average(chain5_ltcy_default))
			# print(np.average(chain6_ltcy_default))
			# print(np.average(chain7_ltcy_default))
			# print(np.average(chain8_ltcy_default))
			

			labels      = (["default", "openmp"])
			colors      = [(220/255.,237/255.,204/255.), (120/255.,214/255.,201/255.)]

			plt.figure(figsize=(30,20))

			bplot_1     = plt.boxplot(chain1, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(1,1.5),widths=0.2)
			for patch, color in zip(bplot_1['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_2     = plt.boxplot(chain2, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(2,2.5),widths=0.2)
			for patch, color in zip(bplot_2['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_3     = plt.boxplot(chain3, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(3,3.5),widths=0.2)
			for patch, color in zip(bplot_3['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_4     = plt.boxplot(chain4, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(4,4.5),widths=0.2)
			for patch, color in zip(bplot_4['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_5     = plt.boxplot(chain5, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(5,5.5),widths=0.2)
			for patch, color in zip(bplot_5['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_6     = plt.boxplot(chain6, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(6,6.5),widths=0.2)
			for patch, color in zip(bplot_6['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_7     = plt.boxplot(chain7, patch_artist=True,labels=labels, showfliers=True, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(7,7.5),widths=0.2)
			for patch, color in zip(bplot_7['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)
			bplot_8     = plt.boxplot(chain8, patch_artist=True,labels=labels, showfliers=T如饿, medianprops={'color':'black', 'linewidth':'1.2'}, positions=(8,8.5),widths=0.2)
			for patch, color in zip(bplot_8['boxes'], colors):
							patch.set_facecolor(color)
							patch.set(linewidth=0.5)


			x_position  = [1, 2, 3, 4, 5, 6, 7, 8]
			x_posn_fmt  = ["Chain1", "Chain2", "Chain3", "Chain4", "Chain5", "Chain6", "Chain7", "Chain8"]

			plt.xticks([i + 0.35 / 2 for i in x_position], x_posn_fmt, fontsize=14)
			plt.yticks(fontsize=10)
			plt.ylabel('Latency [$m$s]', fontsize=13)
			plt.grid(linestyle="--", alpha=0.3)  #绘制图中虚线 透明度0.3
			plt.legend(bplot_1['boxes'],labels,loc='upper left', fontsize=12)  #绘制表示框，右下角绘制
			plt.savefig(fname=lcty_path+last_part, bbox_inches='tight')
			plt.show()