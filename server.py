import socket
import keyboard  # Install using 'pip install keyboard'

HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 1234       # Port to listen on

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

mosfet_state = False  # False = OFF, True = ON

print(f"Server is listening on {HOST}:{PORT}")
print("Press SPACEBAR to toggle MOSFET state.")

while True:
    client_socket, client_address = server_socket.accept()
    # print(f"Connection from {client_address}")

    # Wait for a key press in a non-blocking way
    if keyboard.is_pressed("space"):
        mosfet_state = not mosfet_state  # Toggle state
        print(f"TOGGLED MOSFET: {'ON' if mosfet_state else 'OFF'}")

    response = "TURN ON" if mosfet_state else "TURN OFF"

    # Debugging: Print what the server sends
    # print(f"Sending response: {response}")

    # Ensure there is a slight delay before sending
    client_socket.sendall(response.encode('utf-8'))
    # print(f"Sent: {response}")

    # Close the connection after sending
    client_socket.close()
