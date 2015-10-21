import xmlrpclib

server = xmlrpclib.ServerProxy("http://effbot.org/rpc/echo.cgi")

print "'testing'"
print repr(server.echo("testing"))
print "['testing', 'testing', 1, 2.0, [3]]"
print repr(server.echo("testing", "testing", 1, 2.0, [3]))
