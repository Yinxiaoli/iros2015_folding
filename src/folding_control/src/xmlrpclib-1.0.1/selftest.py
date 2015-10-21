# $Id: //modules/xmlrpclib/selftest.py#5 $
# minimal xmlrpclib selftest (2.1 and later)

import xmlrpclib, string

def roundtrip(data):
    """
    Make sure the given data survives a marshal/unmarshal roundtrip

    >>> roundtrip((1,))
    >>> roundtrip((1L,))
    >>> roundtrip(("1",))
    >>> roundtrip(([], []))
    >>> roundtrip(((), ())) # @XMLRPC11
    ([], [])
    >>> roundtrip(({"one": 1, "two": 2},))
    >>> roundtrip(({},))
    >>> roundtrip((xmlrpclib.DateTime(0), xmlrpclib.True, xmlrpclib.False))
    >>> roundtrip((xmlrpclib.Binary("data"),))
    >>> roundtrip(xmlrpclib.Fault(100, "cans of spam"))
    Traceback (most recent call last):
    Fault: <Fault 100: 'cans of spam'>
    >>> roundtrip(("hello", xmlrpclib.Binary("test"), 1, 2.0, [3, 4, 5]))
    """
    body = xmlrpclib.dumps(data)
    result = xmlrpclib.loads(body)[0]
    if result != data:
        print result

def request_encoding(data, method, encoding=None):
    r"""
    Test http request marshalling

    >>> request_encoding(unicode("abc", "ascii"), "test", "iso-8859-1")
    (((u'abc',), 'test'), (('abc',), 'test'))

    >>> request_encoding(unicode("едц", "iso-8859-1"), "test", "iso-8859-1")
    (((u'\xe5\xe4\xf6',), 'test'), ((u'\xe5\xe4\xf6',), 'test'))

    >>> request_encoding(unicode("едц", "iso-8859-1"), "test")
    (((u'\xe5\xe4\xf6',), 'test'), ((u'\xe5\xe4\xf6',), 'test'))
    """
    if not isinstance(data, type(())):
        data = (data,)
    body = xmlrpclib.dumps(data, method, encoding=encoding)
    return (data, method), xmlrpclib.loads(body)

if __name__ == "__main__":
    import doctest, selftest
    doctest.testmod(selftest)
