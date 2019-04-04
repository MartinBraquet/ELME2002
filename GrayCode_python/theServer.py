#!/usr/bin/python3

import http.server
from queue import *


VERBOSE = False
class MyHandler(http.server.CGIHTTPRequestHandler):

    def printPath(self):
        if VERBOSE:
            print("Path:",self.path)

    def printHeaders(self):
        headers = self.headers
        for h in headers:
            if(VERBOSE):
                print(h," : " , headers[h])

    def do_GET(self):
        self.printPath()
        self.printHeaders()
        global myQueue
        valueToSend = "GET"
        myQueue.put(valueToSend)
        http.server.CGIHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        self.printPath()
        self.printHeaders()
        global myQueue
        valueToSend = "POST"
        myQueue.put(valueToSend)
        http.server.CGIHTTPRequestHandler.do_POST(self)


    def do_PUT(self):
        self.printPath()
        self.printHeaders()
        http.server.CGIHTTPRequestHandler.do_PUT(self)

    def do_HEAD(self):
        self.printPath()
        self.printHeaders()
        http.server.CGIHTTPRequestHandler.do_HEAD(self)

############################################################
############################################################
myQueue = Queue()

def runServer(q):
    global myQueue
    myQueue = q
    PORT = 8888
    server_address = ("", PORT)

    server = http.server.HTTPServer
    handler = MyHandler
    handler.cgi_directories = ["/"]
    print("\t\u2192\tServeur actif sur le port :", PORT)

    httpd = server(server_address, handler)
    httpd.serve_forever()
