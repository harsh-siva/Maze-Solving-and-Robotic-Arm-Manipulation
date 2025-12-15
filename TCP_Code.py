import socket
import time
import csv

def send_tcp_packet(client_socket, message):
    """Send a message to the server and receive the response."""
    try:
        client_socket.sendall(message.encode('utf-8'))
        response = client_socket.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        return None

def read_angles_from_csv(file_path):
    """Read angles from a CSV file."""
    angles = []
    with open(file_path, mode='r', encoding='utf-8') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            # Convert each angle to a float and append to angles list
            angles.append([float(angle) for angle in row])
    return angles

# Read angles from CSV file
angles = read_angles_from_csv('joint_angles.csv')
print(angles)
SERVER_IP = '192.168.1.159'  # Server IP
SERVER_PORT = 5001           # Server port

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((SERVER_IP, SERVER_PORT))

    for angle_set in angles:
    
        rounded_angles = [round(angle, 2) for angle in angle_set]
        message = f"set_angles({', '.join(map(str, rounded_angles))}, 400)"
        send_tcp_packet(client_socket, message)
