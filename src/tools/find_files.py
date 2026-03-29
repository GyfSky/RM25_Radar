import os
import re
import heapq
from datetime import datetime

# 正则表达式匹配文件名格式 RadarMain_{nano}_{timestamp}
FILENAME_PATTERN = re.compile(r'RadarMain_(\d+)_(\d+)')

def parse_filename(filename):
    match = FILENAME_PATTERN.match(filename)
    if not match:
        return None
    nano_str, timestamp_str = match.groups()

    try:
        # 假设 timestamp_str 是毫秒时间戳
        seconds = int(timestamp_str) // 1000
        milliseconds = int(timestamp_str) % 1000
        nano_from_millis = milliseconds * 1_000_000
        nano_total = nano_from_millis + int(nano_str)
        full_timestamp = seconds + nano_total / 1_000_000_000.0
        return full_timestamp
    except ValueError:
        return None

def find_latest_n_files(directory, n=10):
    heap = []
    for filename in os.listdir(directory):
        timestamp = parse_filename(filename)
        if timestamp is not None:
            if len(heap) < n:
                heapq.heappush(heap, (timestamp, filename))
            else:
                if timestamp > heap[0][0]:
                    heapq.heapreplace(heap, (timestamp, filename))
    latest_files = sorted(heap, key=lambda x: x[0], reverse=True)
    return latest_files

if __name__ == '__main__':
    directory = '/home/thesky/.ros/log'  # 可以改为你的目标路径
    n = 500
    latest_files = find_latest_n_files(directory, n)

    if latest_files:
        for ts, fname in latest_files:
            readable_time=datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f UTC+8')
            print(f"找到较新的文件: {fname}")
            print(f"对应时间: {readable_time}")
    else:
        print("未找到符合命名规则的文件。")