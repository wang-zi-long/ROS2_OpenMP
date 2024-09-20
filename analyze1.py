import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates

# 日志数据
log_data = [
    "|TID:1100306176|-->[Chain1-Command3.exe:330(100us)]-->[Start@1726807516482377485-End@1726807516514606120-Cost:32228635(ns) and Received msg: 3].",
    "|TID:1100306176|-->[Chain1-Sensor1.exe:330(100us)]-->[Start@1726807516617563015-End@1726807516649997056-Cost:32434041(ns) and Published msg: 4].",
    "|TID:1203302336|-->[Chain2-Sensor1.exe:410(100us)]-->[Start@1726807516634072624-End@1726807516674497357-Cost:40424733(ns) and Published msg: 3].",
    "|TID:1100306176|-->[Chain1-Transfer2.exe:330(100us)]-->[Start@1726807516650309435-End@1726807516682669103-Cost:32359668(ns) and Route msg: 4].",
    "|TID:1100306176|-->[Chain1-Command3.exe:330(100us)]-->[Start@1726807516682867711-End@1726807516715080018-Cost:32212307(ns) and Received msg: 4].",
    "|TID:1203302336|-->[Chain2-Transfer2.exe:410(100us)]-->[Start@1726807516674783141-End@1726807516715130537-Cost:40347396(ns) and Route msg: 3].",
    "|TID:1100306176|-->[Chain2-Command3.exe:410(100us)]-->[Start@1726807516715224273-End@1726807516755219266-Cost:39994993(ns) and Received msg: 3].",
    "|TID:1100306176|-->[Chain1-Sensor1.exe:330(100us)]-->[Start@1726807516817466796-End@1726807516849866193-Cost:32399397(ns) and Published msg: 5]."
]

# 解析日志数据
def parse_log_fixed(log_entry):
    tid = log_entry.split("|-->")[0].split(":")[1]
    exe_info = log_entry.split("|-->")[1].split(":")[0].strip()
    chain = exe_info.split("-")[0].split("[")[1]  # 获取Chain部分，并移除前括号
    exe = exe_info.split("-")[1].split(".")[0]  # 获取Executable部分
    start_ns = int(log_entry.split("Start@")[1].split("-End")[0])
    end_ns = int(log_entry.split("End@")[1].split("-Cost")[0])
    cost_ns = int(log_entry.split("Cost:")[1].split("(ns)")[0])
    msg_type = log_entry.split("msg: ")[1].strip("].")  # 更准确地提取消息类型
    return {
        "TID": tid,
        "Chain": chain,
        "Executable": exe,
        "Start_ns": start_ns,
        "End_ns": end_ns,
        "Cost_ns": cost_ns,
        "Message": msg_type
    }

# 创建 DataFrame
log_entries_fixed = [parse_log_fixed(entry) for entry in log_data]
df_logs_fixed = pd.DataFrame(log_entries_fixed)

# 将纳秒时间戳转换为 datetime 对象
df_logs_fixed['Start_dt'] = pd.to_datetime(df_logs_fixed['Start_ns'], unit='ns')
df_logs_fixed['End_dt'] = pd.to_datetime(df_logs_fixed['End_ns'], unit='ns')

# 将 datetime 对象转换为 Matplotlib 的日期数字
df_logs_fixed['Start_num'] = mdates.date2num(df_logs_fixed['Start_dt'])
df_logs_fixed['End_num'] = mdates.date2num(df_logs_fixed['End_dt'])

# 重新绘制图表
plt.figure(figsize=(14, 8))

# 绘制每个日志条目的时间跨度
for index, row in df_logs_fixed.iterrows():
    plt.plot([row['Start_num'], row['End_num']], [index] * 2, marker='o', linestyle='-')

# 设置图表标题和坐标轴标签
plt.title('Timeline of Events from Log Data')
plt.xlabel('Time')
plt.ylabel('Event Index')

# 设置x轴的时间显示格式
plt.gca().xaxis.set_major_locator(mdates.AutoDateLocator())
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))

# 设置y轴的标签为可执行文件名
plt.yticks(range(len(df_logs_fixed)), df_logs_fixed['Executable'])

plt.gcf().autofmt_xdate()  # 自动格式化日期标签以防止重叠
plt.grid(True)  # 添加网格线
plt.tight_layout()  # 调整布局
plt.show()  # 显示图表
