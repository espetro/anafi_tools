import socket

hostname, sld, tld, port = 'www', 'integralist', 'co.uk', 80
target = '{}.{}.{}'.format(hostname, sld, tld)

# create an ipv4 (AF_INET) socket object using the tcp protocol (SOCK_STREAM)
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect the client
# client.connect((target, port))
client.connect(('0.0.0.0', 9999))

# send some data (in this case a HTTP GET request)
client.send('GET /index.html HTTP/1.1\r\nHost: {}.{}\r\n\r\n'.format(sld, tld))

# receive the response data (4096 is recommended buffer size)
response = client.recv(4096)

print response

# from __future__ import print_function
# from twisted.python import log
# from twisted.internet import reactor
# from twisted.internet.protocol import ServerFactory
# from twisted.internet.protocol import ServerFactory, Protocol


# import sys
# import socket

# TCP_IP = "localhost"
# TCP_PORT = 9800
# BUFFER_SIZE = 1024

# # if __name__ == "__main__":
# #     # for line in sys.stdin:
# #     #     if "omniscient_pedestrian.worldPosition" in line:
# #     #         print(line)
# #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# #     s.connect((TCP_IP, TCP_PORT))
# #     while True:
# #         response = s.recv(BUFFER_SIZE)
# #         print(response)

# class EchoServerFactory(ServerFactory):
#     def buildProtocol(self, addr):
#         print("Data received")
#         return EchoServerProtocol()

# class EchoServerProtocol(Protocol):
#     def dataReceived(self, data):
#         log.msg('Data received {}'.format(data))
#         self.transport.write(data)

#     def connectionMade(self):
#         log.msg('Client connection from {}'.format(self.transport.getPeer()))

#     def connectionLost(self, reason):
#         log.msg('Lost connection because {}'.format(reason))

# if __name__ == '__main__':
#     log.startLogging(sys.stdout)
#     log.msg('Start your engines...')
#     reactor.listenTCP(TCP_PORT, EchoServerFactory())
#     reactor.run()
