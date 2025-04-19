import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Tạo thư mục để lưu ảnh nếu chưa tồn tại
output_dir = "pid_analysis_results"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Đọc dữ liệu từ file CSV
df = pd.read_csv("robot_log.csv", header=None)

# Gán tên cột tương ứng với dữ liệu
df.columns = [
    "time", "ir0", "ir1", "ir2", "ir3",
    "Kp", "Ki", "Kd", "error", "speedL", "speedR"
]

# Chuyển time từ millis thành giây để dễ quan sát
df["time"] = pd.to_numeric(df["time"], errors='coerce')
df["time_sec"] = df["time"] / 1000.0

# Làm sạch dữ liệu
df = df.dropna()

# Tính các giá trị PID components từ dữ liệu
df["p_term"] = df["Kp"] * df["error"]
df["i_term"] = df["Ki"] * df.get("integral", 0)  # Nếu không có cột integral thì giả sử 0
df["d_term"] = df["Kd"] * df["error"].diff()

# Xuất thông tin cơ bản về dữ liệu
print(f"Số lượng mẫu: {len(df)}")
print(f"Thời gian ghi: {df['time_sec'].max() - df['time_sec'].min():.2f} giây")
print(f"Tần suất lấy mẫu trung bình: {1000 * len(df) / (df['time'].max() - df['time'].min()):.2f} Hz")

# Giảm số lượng điểm để vẽ đồ thị mượt hơn (downsampling)
if len(df) > 10000:
    sampling_rate = len(df) // 10000
    df_plot = df.iloc[::sampling_rate].copy()
else:
    df_plot = df.copy()

# 1. Biểu đồ PID Error và Tốc độ
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True, gridspec_kw={'height_ratios': [1, 1]})

# Error plot
ax1.plot(df_plot["time_sec"], df_plot["error"], label="PID Error", color="red", linewidth=1.5)
ax1.axhline(y=0, color='black', linestyle='-', alpha=0.3)
ax1.axhline(y=2.5, color='green', linestyle='--', label="max_error_for_high_speed", alpha=0.7)
ax1.axhline(y=3.5, color='orange', linestyle='--', label="min_error_for_low_speed", alpha=0.7)
ax1.axhline(y=-2.5, color='green', linestyle='--', alpha=0.7)
ax1.axhline(y=-3.5, color='orange', linestyle='--', alpha=0.7)
ax1.set_ylabel("Error", fontsize=12)
ax1.set_title("PID Error và Tốc độ Động cơ", fontsize=14, fontweight='bold')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Speed plot
ax2.plot(df_plot["time_sec"], df_plot["speedL"], label="Speed Left", color="blue", linewidth=1.5)
ax2.plot(df_plot["time_sec"], df_plot["speedR"], label="Speed Right", color="purple", linewidth=1.5)
ax2.set_xlabel("Thời gian (s)", fontsize=12)
ax2.set_ylabel("PWM Value", fontsize=12)
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "error_and_speed.png"), dpi=300)
plt.close()

# 2. Biểu đồ các thành phần PID
fig, ax = plt.subplots(figsize=(14, 8))
ax.plot(df_plot["time_sec"], df_plot["p_term"], label="P Term", color="red", linewidth=1.5)
ax.plot(df_plot["time_sec"], df_plot["d_term"], label="D Term", color="blue", linewidth=1.5)
if "i_term" in df_plot.columns and df_plot["i_term"].abs().max() > 0:
    ax.plot(df_plot["time_sec"], df_plot["i_term"], label="I Term", color="green", linewidth=1.5)
ax.set_xlabel("Thời gian (s)", fontsize=12)
ax.set_ylabel("Giá trị", fontsize=12)
ax.set_title("Các thành phần PID", fontsize=14, fontweight='bold')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "pid_components.png"), dpi=300)
plt.close()

# 3. Biểu đồ Giá trị cảm biến IR
fig, ax = plt.subplots(figsize=(14, 8))
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
for i in range(4):
    ax.plot(df_plot["time_sec"], df_plot[f"ir{i}"], label=f"IR {i}", color=colors[i], linewidth=1.5)
ax.axhline(y=300, color='black', linestyle='--', label="Black Threshold", alpha=0.7)
ax.set_xlabel("Thời gian (s)", fontsize=12)
ax.set_ylabel("Giá trị IR", fontsize=12)
ax.set_title("Giá trị cảm biến IR", fontsize=14, fontweight='bold')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "ir_sensors.png"), dpi=300)
plt.close()

# 4. Biểu đồ phân tích hiệu suất - Chênh lệch tốc độ và Error
fig, ax1 = plt.subplots(figsize=(14, 8))

# Tính chênh lệch tốc độ
df_plot["speed_diff"] = df_plot["speedL"] - df_plot["speedR"]

color = 'tab:red'
ax1.set_xlabel("Thời gian (s)", fontsize=12)
ax1.set_ylabel("Error", fontsize=12, color=color)
ax1.plot(df_plot["time_sec"], df_plot["error"], color=color, linewidth=1.5, label="Error")
ax1.tick_params(axis='y', labelcolor=color)
ax1.axhline(y=0, color='black', linestyle='-', alpha=0.3)

ax2 = ax1.twinx()
color = 'tab:blue'
ax2.set_ylabel("Chênh lệch tốc độ (L-R)", fontsize=12, color=color)
ax2.plot(df_plot["time_sec"], df_plot["speed_diff"], color=color, linewidth=1.5, label="Speed Diff")
ax2.tick_params(axis='y', labelcolor=color)

# Tạo legend kết hợp
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

ax1.set_title("Mối quan hệ giữa Error và Chênh lệch tốc độ", fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "error_vs_speed_diff.png"), dpi=300)
plt.close()

# 5. Biểu đồ phổ tần số của Error (để phân tích dao động)
from scipy import signal

if len(df) > 100:  # Cần đủ điểm để phân tích tần số
    # Lấy mẫu với tần số đều đặn
    sample_rate = 1000 / 5  # 5ms/mẫu = 200Hz
    
    # Áp dụng FFT
    error_signal = df["error"].values
    f, Pxx = signal.welch(error_signal, fs=sample_rate, nperseg=1024)
    
    plt.figure(figsize=(14, 6))
    plt.semilogy(f, Pxx)
    plt.xlabel('Tần số (Hz)', fontsize=12)
    plt.ylabel('Mật độ phổ công suất', fontsize=12)
    plt.title('Phân tích tần số của Error', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "error_frequency.png"), dpi=300)
    plt.close()

print(f"Đã xuất các biểu đồ vào thư mục '{output_dir}'")

# 6. Biểu đồ Kp, Ki, Kd theo thời gian - tách riêng để tiện quan sát
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

# Kp plot
ax1.plot(df_plot["time_sec"], df_plot["Kp"], color="red", linewidth=1.5)
ax1.set_ylabel("Kp", fontsize=12)
ax1.set_title("Thông số Kp theo thời gian", fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)

# Ki plot
ax2.plot(df_plot["time_sec"], df_plot["Ki"], color="green", linewidth=1.5)
ax2.set_ylabel("Ki", fontsize=12)
ax2.set_title("Thông số Ki theo thời gian", fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)

# Kd plot
ax3.plot(df_plot["time_sec"], df_plot["Kd"], color="blue", linewidth=1.5)
ax3.set_xlabel("Thời gian (s)", fontsize=12)
ax3.set_ylabel("Kd", fontsize=12)
ax3.set_title("Thông số Kd theo thời gian", fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "pid_parameters_separate.png"), dpi=300)
plt.close()
