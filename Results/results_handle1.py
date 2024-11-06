import matplotlib.pyplot as plt

x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
y1 = []  # 确保数据点数量与x相同
y2 = []  # 确保数据点数量与x相同
y3 = []  # 确保数据点数量与x相同
y4 = []  # 确保数据点数量与x相同

i = 1
while i < 11 :
    with open("/home/neu/Results/Callback_num/{0}_CPU.log".format(i), 'r') as file:
        lines = file.readlines()
    last_line = lines[-1]
    columns = last_line.split()
    y1.append((float)(columns[7]))

    with open("/home/neu/Results/Callback_num_new/{0}_CPU.log".format(i), 'r') as file:
        lines = file.readlines()
    last_line = lines[-1]
    columns = last_line.split()
    y3.append((float)(columns[7]))

    with open("/home/neu/Results/Callback_num/{0}_CS.log".format(i), 'r') as file:
        lines = file.readlines()
    total = 0
    j = -1
    while j >= -16 :
        columns = lines[j].split()
        if columns[3] != "0.00" :
            total += (float)(columns[3])
        j = j - 1
    y2.append(total)
    
    with open("/home/neu/Results/Callback_num_new/{0}_CS.log".format(i), 'r') as file:
        lines = file.readlines()
    total = 0
    j = -1
    while j >= -4 :
        columns = lines[j].split()
        if columns[3] != "0.00" :
            total += (float)(columns[3])
        j = j - 1
    y4.append(total)

    i = i + 1

fig, ax1 = plt.subplots()  # 正确捕获返回值

ax1.plot(x, y1, 'g-o', label='GOMP-CPU')  # 在ax1上绘制
ax1.plot(x, y3, 'r-o', label='ROSOMP-CPU')  # 在ax1上绘制
ax1.set_xlabel('Callback Number', fontsize=12)
ax1.set_ylabel('CPU Utilization(%)', color='g', fontsize=14, fontweight='bold')

ax2 = ax1.twinx()  # 在相同的x轴上创建第二个y轴
ax2.plot(x, y2, 'b-s', label='GOMP-CS')
ax2.plot(x, y4, 'y-s', label='ROSOMP-CS')
ax2.set_ylabel('Total CS Number', color='b', fontsize=14, fontweight='bold')

ax1.set_xticks(x)

lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left', fontsize='large')
ax1.grid(True,linestyle="--", alpha=0.5, which='major')
ax2.grid(True,linestyle="--", alpha=0.5, which='major')
plt.show()
