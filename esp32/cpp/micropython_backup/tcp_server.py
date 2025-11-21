import socket
import select

class TCPServer:
    def __init__(self, port=8888):
        self.port = port
        self.server_socket = None
        self.clients = []
        self.client_buffers = {}
        self.running = False

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(2)
            self.server_socket.setblocking(False)
            self.running = True
            print("TCP server listening on port", self.port)
            return True
        except Exception as e:
            print("TCP server start failed:", e)
            return False

    def accept_clients(self):
        if not self.running:
            return
        try:
            client, addr = self.server_socket.accept()
            client.setblocking(False)
            self.clients.append(client)
            self.client_buffers[client] = b""
            print("TCP: client connected from", addr)
            print("TCP: total clients =", len(self.clients))
            try:
                client.send(b"WELCOME_FROM_ESP32\n")
            except Exception as e:
                print("TCP: failed to send welcome:", repr(e))
        except OSError:
            pass
        except Exception as e:
            print("TCP: accept_clients error:", repr(e))

    def send_to_all(self, message):
        if not self.running:
            return
        dead_clients = []
        for client in self.clients:
            try:
                client.send((message + "\n").encode())
            except Exception:
                dead_clients.append(client)
        for client in dead_clients:
            self._remove_client(client)

    def try_read_line(self):
        if not self.clients:
            return None
        
        for client in self.clients[:]:
            try:
                data = client.recv(256)
                if not data:
                    self._remove_client(client)
                    continue
                
                print("TCP: recv'd", len(data), "bytes:", repr(data[:50]))
                
                self.client_buffers[client] += data
                buf = self.client_buffers[client]
                
                if b"\n" in buf or b"\r" in buf:
                    nl = buf.find(b"\n")
                    cr = buf.find(b"\r")
                    if nl != -1 and cr != -1:
                        idx = min(nl, cr)
                    else:
                        idx = nl if nl != -1 else cr
                    
                    line = buf[:idx]
                    self.client_buffers[client] = buf[idx+1:]
                    
                    try:
                        decoded = line.decode('utf-8').strip()
                        print("TCP: decoded line:", repr(decoded))
                        return decoded
                    except:
                        return ''
                
            except OSError as e:
                if e.args[0] not in (11, 35, 115):
                    print("TCP: OSError", e.args[0])
                continue
            except Exception as e:
                print("TCP read error:", e)
                self._remove_client(client)
        
        return None

    def _remove_client(self, client):
        try:
            if client in self.clients:
                self.clients.remove(client)
            if client in self.client_buffers:
                del self.client_buffers[client]
            client.close()
            print("TCP: Client disconnected")
        except Exception:
            pass

    def stop(self):
        for client in self.clients:
            try:
                client.close()
            except Exception:
                pass
        self.clients = []
        self.client_buffers = {}
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
        self.running = False