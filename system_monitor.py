#!/usr/bin/env python3
import psutil
import time
import subprocess
import os
from datetime import datetime

def get_process_info(process_name="scservo_node"):
    """获取指定进程的详细信息"""
    process_info = []
    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
        try:
            if process_name in proc.info['name']:
                proc_info = {
                    'pid': proc.info['pid'],
                    'cpu_percent': proc.info['cpu_percent'],
                    'memory_percent': proc.info['memory_percent'],
                    'threads': proc.num_threads(),
                    'status': proc.status()
                }
                process_info.append(proc_info)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return process_info

def get_usb_info():
    """获取USB设备信息"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        return result.stdout
    except Exception as e:
        return f"Error getting USB info: {str(e)}"

def get_serial_latency():
    """获取串口延迟设置"""
    try:
        with open('/sys/bus/usb-serial/devices/ttyUSB0/latency_timer', 'r') as f:
            return f.read().strip()
    except Exception as e:
        return f"Error reading latency timer: {str(e)}"

def monitor_system(interval=1):
    """监控系统状态"""
    print(f"开始监控系统状态 - {datetime.now()}")
    print("\n初始USB设备信息:")
    print(get_usb_info())
    print(f"\n串口延迟设置: {get_serial_latency()}ms")
    
    print("\n实时监控中...")
    print("时间戳 CPU使用率 内存使用率 CPU负载 scservo_node进程信息")
    
    start_time = time.time()
    while True:
        cpu_percent = psutil.cpu_percent(interval=None)
        mem_percent = psutil.virtual_memory().percent
        load_avg = os.getloadavg()
        
        process_info = get_process_info()
        
        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f"\n{timestamp}")
        print(f"CPU使用率: {cpu_percent}%")
        print(f"内存使用率: {mem_percent}%")
        print(f"CPU负载(1,5,15分钟): {load_avg}")
        
        if process_info:
            for proc in process_info:
                print(f"scservo_node进程信息:")
                print(f"  PID: {proc['pid']}")
                print(f"  CPU使用率: {proc['cpu_percent']}%")
                print(f"  内存使用率: {proc['memory_percent']}%")
                print(f"  线程数: {proc['threads']}")
                print(f"  状态: {proc['status']}")
        else:
            print("未找到 scservo_node 进程")
            
        # 获取 CPU 频率信息
        cpu_freq = psutil.cpu_freq()
        if cpu_freq:
            print(f"CPU频率: 当前={cpu_freq.current:.1f}MHz, "
                  f"最小={cpu_freq.min:.1f}MHz, "
                  f"最大={cpu_freq.max:.1f}MHz")
            
        # IO 统计
        disk_io = psutil.disk_io_counters()
        if disk_io:
            print(f"磁盘IO: 读取={disk_io.read_bytes/1024/1024:.1f}MB, "
                  f"写入={disk_io.write_bytes/1024/1024:.1f}MB")
            
        time.sleep(interval)

if __name__ == "__main__":
    print("系统监控工具")
    print("=" * 50)
    monitor_system()
