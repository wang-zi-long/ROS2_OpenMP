import matplotlib.pyplot as plt

x = [4, 5, 6, 7, 8]
y1 = []  # 确保数据点数量与x相同
y2 = []  # 确保数据点数量与x相同

i = 4
while i < 9:
    with open("/home/neu/Results/Cores/{0}_CPU.log".format(i), 'r') as file:
        lines = file.readlines()
    last_line = lines[-1]
    columns = last_line.split()
    y1.append(float(columns[7]))

    with open("/home/neu/Results/Cores/{0}_CS.log".format(i), 'r') as file:
        lines = file.readlines()
    total = 0
    j = -1
    while j >= (i*i*(-1)):
        columns = lines[j].split()
        total += float(columns[3])
        j -= 1
    y2.append(total)
    i += 1

fig, ax1 = plt.subplots()

ax1.plot(x, y1, 'g-o', label='CPU Utilization')
ax1.set_xlabel('Executor Thread Number', fontsize=12)
ax1.set_ylabel('CPU Utilization(%)', color='g', fontsize=14, fontweight='bold')

max_y1 = max(y1)
ax1.set_ylim(0, max_y1 * (1400/16))

ax2 = ax1.twinx()
ax2.plot(x, y2, 'b-s', label='Total CS Number')
ax2.set_ylabel('Total CS Number', color='b', fontsize=14, fontweight='bold')

max_y2 = max(y2)
ax2.set_ylim(0, max_y2)

ax1.set_xticks(x)  # 设置X轴刻度

# 获取并设置图例
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left', fontsize='large')

# 添加网格
ax1.grid(True, linestyle="--", alpha=0.5, which='major')
# ax2.grid(True, linestyle="--", alpha=0.5, which='major')

plt.show()
