from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib, os


class Client():
	def __init__(self, ip_addr, port):
		self.proxy = xmlrpclib.ServerProxy("http://"+str(ip_addr) + ":" + str(port) +"/")
		print "communication established"

	def launch(self):
		print self.proxy.launch()

	#Receive file from server
	def recv_file(self, fname):
		with open(fname, "wb") as handle:
			handle.write(self.proxy.send_file().data)
	#Send file to server
	def send_file(self, fname):
		with open(fname, "rb") as handle:
			send_data = xmlrpclib.Binary(handle.read())
			self.proxy.recv_file(send_data)
	#If the receive file is ready
	def is_ready(self):
		return self.proxy.is_ready()
			
class Server():
	recv_fname = 'temp'
	send_fname = 'temp'

	def __init__(self):
		pass

	def is_ready(self):
		return os.path.isfile(self.send_fname)

	def set_recv_file_name(self, name):
		self.recv_fname = name
	def set_send_file_name(self, name):
		self.send_fname = name
	
	def recv_file(self, f):
		with open(self.recv_fname, "wb") as handle:
			handle.write(f.data)
		return True

	def send_file(self):
		with open(self.send_fname, "rb") as handle:
			return xmlrpclib.Binary(handle.read())

	def Serve(self,ip_addr, port, functions = dict()): 
		server = SimpleXMLRPCServer((ip_addr, int(port)))
		server.register_function(self.recv_file, 'recv_file')
		server.register_function(self.send_file, 'send_file')
		server.register_function(self.is_ready, 'is_ready')
		
		for f_name in functions:
			server.register_function(functions[f_name], f_name)
		print "Listening on port " + str(port) + "..."
		server.serve_forever()

def main():
	c = Server()
	c.set_recv_file_name('file')
	c.Serve('localhost', 8000)

if __name__ == "__main__":
	exit(main())
