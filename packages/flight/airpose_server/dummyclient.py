import socket
import sys
import numpy as np
import time

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('129.69.124.204', 9911)
print(sys.stderr, 'connecting to %s port %s' % server_address)
sock.connect(server_address)

img_data = np.random.rand(224*224*3).astype(np.uint8)
p_data = np.random.rand(3).astype(np.float32)
p_data.dtype = np.uint8
stage = np.array(0).reshape(1).astype(np.uint8)
message = memoryview(np.concatenate([stage,p_data,img_data]))
t1 = time.time()
sock.sendall(message)

data = sock.recv(136*4)
data_np = np.frombuffer(data,dtype=np.float32,count=136)

print("stage0 eval time: %s" % (time.time()-t1))

data_np.dtype = np.uint8

message = memoryview(np.concatenate([np.array(1).reshape(1).astype(np.uint8),data_np]))
t1 = time.time()
sock.sendall(message)

data = sock.recv(136*4)
data_np2 = np.frombuffer(data,dtype=np.float32,count=136)

print("stage1 eval time: %s" % (time.time()-t1))

data_np2.dtype=np.uint8
message = memoryview(np.concatenate([np.array(2).reshape(1).astype(np.uint8),data_np2]))
t1 = time.time()
sock.sendall(message)

data = sock.recv(145*4)
data_np3 = np.frombuffer(data,dtype=np.float32,count=136)

print("stage2 eval time: %s" % (time.time()-t1))


