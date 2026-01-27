import socket

# 连接ESP32
esp32_ip = "192.168.0.17"  # ESP32的IP地址
esp32_port = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((esp32_ip, esp32_port))

# 发送GPIO控制命令
command = bytes([0x01, 0x01, 0x00, 0xFF, 0x01])  # GPIO控制
sock.send(command)
command = bytes([0x01, 0xFF, 0xFF, 0xFF, 0xFF])  # GPIO控制
sock.send(command)

# 立即接收返回的GPIO状态（同一连接）
gpio_states = sock.recv(4)
print(f"GPIO状态: {list(gpio_states)}")

# 可以继续发送更多命令，无需重连
command2 = bytes([0x01, 0x00, 0x00, 0x00, 0x00])
sock.send(command2)
command2 = bytes([0x01, 0xFF, 0xFF, 0xFF, 0xFF])  # GPIO控制
sock.send(command2)
gpio_states2 = sock.recv(4)
print(f"GPIO状态2: {list(gpio_states2)}")

sock.close()