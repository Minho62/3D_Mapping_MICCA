import numpy as np
import pandas as pd
import ezdxf
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

# 터미널에서 파일 경로 입력받기
file_path = input("CSV 파일의 경로를 입력하세요: ")

# 앞뒤에 붙은 따옴표 제거
file_path = file_path.strip("'\"")
s = pd.read_csv(file_path)

# 엑셀파일 pandas를 통해 DataFrame만듦 
df = pd.DataFrame(s)
# DataFrame에서 3열까지 추출
df = df.iloc[:, :3]

# 열 이름 x, y, z로 바꾸기 
df.columns = ['x', 'y', 'z']

# x, y, z 최단거리 구하기
x = (df['x'].max() - df['x'].min())
y = (df['y'].max() - df['y'].min())
z = (df['z'].max() - df['z'].min())
print([x, y, z])

# 3D 그래프 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# x, y, z 축에 데이터프레임의 각 열을 매핑하여 3D 산점도 그리기
ax.scatter(df['x'], df['y'], df['z'], c='blue', marker='o', zorder=1, s=0.1)

# 축 이름 설정
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# 그래프 보여주기
plt.show()

# 데이터 준비
x = df['x'].values
y = df['y'].values
z = df['z'].values

# 점 좌표를 numpy 배열로 결합
points = np.column_stack((x, y, z))

# Convex Hull 계산
hull = ConvexHull(points)
volume = hull.volume
print(volume)

# txt 파일로 volume 값 저장 
desktop_path = os.path.join(os.path.expanduser("~"), "Desktop", "volume_txt")

# 폴더가 없으면 생성
if not os.path.exists(desktop_path):
    os.makedirs(desktop_path)

# txt 파일 경로 지정, 확장자 ".txt" 명시
txt_file_path = os.path.join(desktop_path, "volume.txt")

# volume 값을 txt 파일에 저장
with open(txt_file_path, "w") as file:
    file.write(f"Volume: {volume} m^3")