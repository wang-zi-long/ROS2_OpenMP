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

mkdir_path = "/home/neu/Desktop/OpenMP_test/results"

def run_ros_command(i):
	thread_id = threading.get_ident()
	ros_command = "ros2 run intra_process_demo two_node_pipeline {0} > ".format(i) + mkdir_path + "/{0}.log".format(i)
	os.system(ros_command)

def run_cpu_command(i, pid):
	cpu_command = "pidstat -u -p {0} 1 180 > ".format(pid) + mkdir_path + "/{0}_CPU.log".format(i)
	os.system(cpu_command)

def run_cpu_command1(i, pid):
	cpu_command1 = "mpstat -P 0,1 1 180 > " + mkdir_path + "/{0}_CPU1.log".format(i)
	os.system(cpu_command1)

def run_cs_command(i, pid):
	cs_command = "pidstat -w -p {0} 1 180 > ".format(pid) + mkdir_path + "/{0}_CS.log".format(i)
	os.system(cs_command)

def run_log(i):

	# 创建并启动线程执行ros_command
	ros_thread = Thread(target=run_ros_command, args=(i,))
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

	cpu_thread = Thread(target=run_cpu_command, args=(i, (int)(result)))
	cpu_thread.start()

	cpu_thread1 = Thread(target=run_cpu_command1, args=(i, (int)(result)))
	cpu_thread1.start()

	cs_thread = Thread(target=run_cs_command, args=(i, (int)(result)))
	cs_thread.start()

	time.sleep(200)

    # 终止two_node_pipeline进程
	os.system("killall two_node_pipeline")

    # 等待线程结束
	ros_thread.join()
	cpu_thread.join()
	cpu_thread1.join()
	cs_thread.join()

def main():
	i = 1
	while i <= 18 :
		# folder_path = mkdir_path + "/{0}".format(i)
		# os.makedirs(folder_path, exist_ok=True)
		run_log(i)
		i += 1
	

if __name__ == "__main__":
    main()