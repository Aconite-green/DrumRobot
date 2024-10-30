# Receive -> 시간, CAN ID, Actual Pos, Current
# Send -> 시간, CAN ID + 100, Desire Pos, error

import pandas as pd
import matplotlib.pyplot as plt

# TXT 파일 로드 함수 (구분자 ','를 사용)
def load_txt(file_path):
    return pd.read_csv(file_path, delimiter=',', header=None, names=['시간', 'CAN ID', '현재 위치', '전류값'])

# CAN ID별 그래프 그리기 함수 (각각 별도의 팝업)
def plot_by_can_id_receive(df, y_column, y_label):
    for can_id in df['CAN ID'].unique():
        can_id_df = df[df['CAN ID'] == can_id]
        
        # 각각의 CAN ID에 대한 그래프 팝업 생성
        plt.figure(figsize=(10, 6))
        plt.plot(can_id_df['시간'], can_id_df[y_column], label=f'CAN ID {can_id}', marker='o')
        
        plt.xlabel('시간') 
        plt.ylabel(y_label)
        plt.title(f'{y_label} - CAN ID {can_id}')
        plt.legend()
        plt.grid(True)
        plt.show()
        
def plot_by_can_id_send(df, y_column, y_label):
    for can_id in df['CAN ID'].unique():
        can_id_df = df[df['CAN ID'] == can_id]
        
        # 각각의 CAN ID에 대한 그래프 팝업 생성
        plt.figure(figsize=(10, 6))
        plt.plot(can_id_df['시간'], can_id_df[y_column], label=f'CAN ID {can_id}', marker='o')
        
        plt.xlabel('시간') 
        plt.ylabel(y_label)
        plt.title(f'{y_label} - CAN ID {can_id}')
        plt.legend()
        plt.grid(True)
        plt.show()

# 메인 함수
def main():
    # TXT 파일 경로
    file_path = '/home/shy/DrumRobot/DataPlot/data.txt'  # TXT 파일 경로 (수동으로 경로를 지정해주어야 합니다)
    
    # 데이터 로드
    df = load_txt(file_path)
    
    # 각 CAN ID별로 현재 위치 그래프를 따로 그리기
    plot_by_can_id(df, '현재 위치', '현재 위치', 'position')

    # 각 CAN ID별로 전류값 그래프를 따로 그리기
    plot_by_can_id(df, '전류값', '전류값', 'current')

if __name__ == '__main__':
    main()
