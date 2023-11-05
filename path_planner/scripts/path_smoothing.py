import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

# 파일 경로
input_file = "/home/ubuntu/hyjoe/src/path_planner/path/path.txt"
output_file = "smoothed_path.txt"

# 파일 읽기
with open(input_file, 'r') as file:
    lines = file.readlines()

# x, y 좌표 추출
x = []
y = []
for line in lines:
    parts = line.split()
    if len(parts) == 2:
        x.append(float(parts[0]))
        y.append(float(parts[1]))

# Low pass filter를 위한 함수 정의
def low_pass_filter(data, cutoff_freq, fs=1.0, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

# Low pass filter 적용
cutoff_frequency = 0.005  # 이 값을 조정하여 필터 강도 조절
smoothed_y = low_pass_filter(y, cutoff_frequency)

# 보간된 데이터를 파일에 저장
with open(output_file, 'w') as file:
    for i in range(len(x)):
        file.write(f"{x[i]}\t{smoothed_y[i]}\n")

print(f"부드럽게 만들어진 데이터가 {output_file} 파일에 저장되었습니다.")

# 결과를 그래프로 표시
plt.figure()
plt.plot(x, y, label='Original Data', alpha=0.5)
plt.plot(x, smoothed_y, label='Smoothed Data', linewidth=2)
plt.legend()
plt.show()