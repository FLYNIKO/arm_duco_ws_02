import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.0.17", 8080))

# 控制电机0运行
motor0_cmd = bytes([
    0x02,           # 命令类型: 步进电机
    0x00,           # 电机编号: 0
    0x00,           # 方向: 正转
    0x00, 0x05,     # 脉冲宽度: 5μs
    0x01, 0xF4,     # 脉冲间隔: 500μs
    0x00, 0x00, 0x03, 0xE8  # 步数: 1000
])
sock.send(motor0_cmd)
response = sock.recv(3)
print(f"电机0启动: {list(response)}")

# 同时控制电机1运行
motor1_cmd = bytes([
    0x02,           # 命令类型
    0x01,           # 电机编号: 1
    0x01,           # 方向: 反转
    0x00, 0x0A,     # 脉冲宽度: 10μs
    0x00, 0xC8,     # 脉冲间隔: 200μs (更快)
    0x00, 0x00, 0x01, 0xF4  # 步数: 500
])
sock.send(motor1_cmd)

# 查询所有电机状态
query_all = bytes([0x03, 0x03])
sock.send(query_all)
status = sock.recv(2 + 3)  # 2字节头 + 3个电机状态
print(f"所有电机状态: {list(status[2:])}")

# 急停所有电机和GPIO
emergency = bytes([0x00])
sock.send(emergency)

# 停止单个电机 (电机1)
stop_motor1 = bytes([0x03, 0x01, 0x01])
sock.send(stop_motor1)

# 查询所有电机状态
query_all = bytes([0x03, 0x03])
sock.send(query_all)
status = sock.recv(2 + 3)  # 2字节头 + 3个电机状态
print(f"所有电机状态: {list(status[2:])}")

sock.close()