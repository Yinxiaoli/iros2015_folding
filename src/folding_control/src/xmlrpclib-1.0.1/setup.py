#!/usr/bin/env python
#
# Setup script
# $Id: //modules/xmlrpclib/setup.py#4 $
#
# Usage: python setup.py install
#

from distutils.core import setup, Extension

setup(
    name="xmlrpclib",
    version="1.0.1",
    author="Fredrik Lundh",
    author_email="fredrik@pythonware.com",
    maintainer="PythonWare",
    maintainer_email="info@pythonware.com",
    description="xmlrpclib -- an XML-RPC library for Python",
    py_modules = ["xmlrpclib", "SimpleXMLRPCServer"],
    scripts = ["xmlrpc_handler.py", "xmlrpcserver.py", "echotest.py"],
    )
