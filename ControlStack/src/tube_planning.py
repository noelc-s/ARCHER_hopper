import yaml
import socket
import struct
import numpy as np

def get_send(client_socket, format_str):
    def send_z_v(z, v):
        data = struct.pack(f"{z.size}f{v.size}f", *z.reshape((-1,)).tolist(), *v.reshape((-1,)).tolist())
        print("Sending z, v: ", z, v)
        client_socket.sendall(data)
    return send_z_v


def get_receive(client_socket):
    def receive_pz_x():
        data = client_socket.recv(16)
        zx, zy, pz_x, pz_y = struct.unpack('4f', data)
        print("Recieved zx, zy: ", zx, zy, pz_x, pz_y)
        return np.array([zx, zy]), np.array([pz_x, pz_y])
    return receive_pz_x


def main():
    # Read the setup file
    with open('ControlStack/config/gains.yaml') as f:
        data = yaml.safe_load(f)
    N = data["MPC"]["N"]
    dt = data["MPC"]["rom"]["dt"]

    # Initialize Socket Connection
    server_address = ('127.0.0.1', 8081)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(server_address)

    # Initialize Connection:
    client_socket.sendall(struct.pack('!?', True))
    
    send_z_v = get_send(client_socket, "!" + "f" * (N * 4 + 2))
    recieve_pz_x = get_receive(client_socket)

    terminated = False
    try:
        while not terminated:
            z, pz_x = recieve_pz_x()
            v = np.array([0.2, 0.])
            send_z_v(np.repeat((z + v * dt)[None, :], N + 1, axis=0), np.repeat(v[None, :], N, axis=0))
    finally:
        client_socket.close()

    

if __name__ == "__main__":
    main()
