import os
from threading import Thread, get_ident
import trace
import random
import math
import time
import argparse
from itertools import combinations
from pandas import DataFrame
import sys
import subprocess
import threading

def run_ros_command(chain_num, period, exe_ratio, is_openmp):
	thread_id = threading.get_ident()
	ros_command = "ros2 run intra_process_demo two_node_pipeline {0} {1} {2} {3} > /home/neu/Desktop/OpenMP/results/Random5/{0}_{1}_{2}/{3}.log".format(chain_num, period, exe_ratio, is_openmp)
	os.system(ros_command)

def run_cpu_command(chain_num, period, exe_ratio, is_openmp, pid):
	cpu_command = "pidstat -u -p {0} 1 180 > /home/neu/Desktop/OpenMP/results/Random5/{1}_{2}_{3}/{4}_CPU.log".format(pid, chain_num, period, exe_ratio, is_openmp)
	os.system(cpu_command)

def run_cpu_command1(chain_num, period, exe_ratio, is_openmp, pid):
	cpu_command1 = "mpstat -P 0,1 1 180 > /home/neu/Desktop/OpenMP/results/Random5/{1}_{2}_{3}/{4}_CPU1.log".format(pid, chain_num, period, exe_ratio, is_openmp)
	os.system(cpu_command1)

def run_cs_command(chain_num, period, exe_ratio, is_openmp, pid):
	cs_command = "pidstat -w -p {0} 1 180 > /home/neu/Desktop/OpenMP/results/Random5/{1}_{2}_{3}/{4}_CS.log".format(pid, chain_num, period, exe_ratio, is_openmp)
	os.system(cs_command)

def run_topic_command(chain_num, period, exe_ratio, is_openmp, i):
	if i < 3 :
		topic_command = "./topic_hz.sh {0} {1} {2} {3} {4}".format(chain_num, period, exe_ratio, is_openmp, i)
	elif i < 5 :
		topic_command = "./topic_hz.sh {0} {1} {2} {3} {4}".format(chain_num, period, exe_ratio, is_openmp, i)
	else :
		topic_command = "./topic_hz.sh {0} {1} {2} {3} {4}".format(chain_num, period, exe_ratio, is_openmp, i)
	os.system(topic_command)

def run_log(chain_num, period, exe_ratio, is_openmp):

	# 创建并启动线程执行ros_command
	ros_thread = Thread(target=run_ros_command, args=(chain_num, period, exe_ratio, is_openmp))
	ros_thread.start()

	# python脚本的滞后性，必须等几秒后才能识别出来two_node_pipeline线程
	time.sleep(5)

	# 执行命令并捕获输出
	command = "ps aux | grep intra_process_demo/two_node_pipeline | awk '{print $2}'"
	process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

	# 读取输出
	output, error = process.communicate()

	# 检查是否有错误输出
	if error:
		print(f"Error: {error.decode()}")
	else:
		# 获得two_node_pipeline线程的PID
		result = output.decode().strip().split("\n")[0]

	cpu_thread = Thread(target=run_cpu_command, args=(chain_num, period, exe_ratio, is_openmp, (int)(result)))
	cpu_thread.start()

	cpu_thread1 = Thread(target=run_cpu_command1, args=(chain_num, period, exe_ratio, is_openmp, (int)(result)))
	cpu_thread1.start()

	cs_thread = Thread(target=run_cs_command, args=(chain_num, period, exe_ratio, is_openmp, (int)(result)))
	cs_thread.start()

	global_threads = {}
	i = 0
	while i < chain_num :
		global_threads[i] = Thread(target=run_topic_command, args=(chain_num, period, exe_ratio, is_openmp, i + 1))
		global_threads[i].start()
		time.sleep(1)
		i += 1

	time.sleep(240)

    # 终止two_node_pipeline进程
	os.system("killall two_node_pipeline")
	os.system("killall topic_hz.sh")

    # 等待线程结束
	ros_thread.join()
	cpu_thread.join()
	cpu_thread1.join()
	cs_thread.join()
	for thread in global_threads.values():
		thread.join() 

def main():

	i = 5
	while i < 8 :
		# # 需要随机选择四个参数：任务链个数（2～8）、每条任务链的周期（50ms~500ms）、任务链执行周期的比例（暂定0~2）
		# chain_num = random.randint(2, 6)
		# # 任务链周期是50的倍数
		# period	  = random.choice(list(range(50, 501, 50)))
		# exe_ratio = random.randint(0, 2)

		chain_num = i
		period = 200
		exe_ratio = 0
		
		print("chain_num:", chain_num)
		print("period:", period)
		print("exe_ratio:", exe_ratio)
		
		folder_path = "/home/neu/Desktop/OpenMP/results/Random5/{0}_{1}_{2}".format(chain_num, period, exe_ratio)
		while os.path.exists(folder_path):

			chain_num = random.randint(2, 8)
			period	  = random.choice(list(range(50, 501, 50)))
			exe_ratio = random.randint(0, 2)
			
			print("chain_num:", chain_num)
			print("period:", period)
			print("exe_ratio:", exe_ratio)

			# 如果文件夹存在，重新选择随机参数并创建文件夹
			folder_path = "/home/neu/Desktop/OpenMP/results/Random5/{0}_{1}_{2}".format(chain_num, period, exe_ratio)
		
		os.makedirs(folder_path, exist_ok=True)

		run_log(chain_num, period, exe_ratio, 0)
		run_log(chain_num, period, exe_ratio, 1)

		i += 1
	

if __name__ == "__main__":
    main()