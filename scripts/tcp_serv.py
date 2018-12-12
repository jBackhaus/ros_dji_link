#!/usr/bin/env python
#Created by Jan Backhaus on 12.12.18 as part of the project ros_dji_link.

import rospy
import socket
import std_msgs.msg
from std_msgs.msg import String
from threading import Thread
import time


#Byte-Transformation-Functions
#To vend & receive data via TCP-Connection

def bytes2int(str):
    #print "bytes2int: input ",str, ", encoded string ",str.encode('hex')
    return int(str.encode('hex'), 16)

def int2hex(i):
    return hex(i)

def int2bytes(i):
    #print "int2bytes: input ",i
    h = int2hex(i)
    #print "encoded hex:", h
    return hex2bytes(h)

def hex2bytes(h):
    if len(h) > 1 and h[0:2] == '0x':
        h = h[2:]

    num = 8 - len(h)
    while num > 0:
        h="0" + h
        num -= 1

    if len(h) % 2:
        h = "0" + h
    #print "transformed hex: ", h
    return h.decode('hex')


#import parameters

TCP_IP = rospy.get_param('tcpIP')
TCP_PORT = rospy.get_param('tcpPort')
FOLDER = rospy.get_param('folderPath')
BUFFER_SIZE = 1024
CONNECTED = False


#Function to handle connections. This will be used to create threads
def clientthread(conn, publisher):
    filnr = 0
    #infinite loop so that function do not terminate and thread do not end.
    while not rospy.is_shutdown():
        
        #Receiving from client
        #receive length of filename as 4-byte Integer
        data = conn.recv(4)
        if not data: 
            break
        flen = bytes2int(data)

        #receive filename as String of n Bytes
        data = conn.recv(flen)
        if not data: 
            break    
        filename = data
        print "Filename:", filename
        filnr += 1
        path = FOLDER+filename
        file = open(path, "wb")
        
        #send received filename as confirmation
        conn.send(filename)

        #receive filesize as 4-byte Integer
        data = conn.recv(4)
        if not data: 
            break    
        filesize = bytes2int(data)
        print "Filesize:", filesize

        #send received filesize as confirmation
        conn.send(int2bytes(filesize))

        remaining = filesize

        #receive File in 1024-byte chunks
        while remaining > 0:    
            if remaining > 1024:
                #not last chunk
                data = conn.recv(1024, socket.MSG_WAITALL)
                #write bytes to disk
                file.write(data)
                remaining -= 1024
            else:
                #receive last chunk
                data = conn.recv(remaining, socket.MSG_WAITALL)
                #write bytes to disk and close file
                file.write(data)
                file.close()
                remaining = 0
            if not data: 
                break    


        print "Received: ",path
        print ""
        #publish path to saved file in ROS
        publisher.publish(path)
        
        #send number of received bytes as confirmation
        conn.send(int2bytes(filesize))

        #ready to receive next file



            
    
    
    #came out of loop, close connection as rospy is shutting down
    CONNECTED = False
    conn.close()


#main function
def tcp_serv():
    #register publisher
    pub = rospy.Publisher("dji_img_file", String, queue_size=10)
    rospy.init_node('tcpReceiver', anonymous=True)

    #open TCP-socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((TCP_IP, TCP_PORT))
    except socket.error as msg:
        s.close
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        rospy.signal_shutdown("Port already in Use")
        return
        
    #listen for connection
    s.listen(1)
    print 'Listening on {}:{}'.format(TCP_IP, TCP_PORT)


    #wait to accept a connection - blocking call
    conn, addr = s.accept()
    print 'Connected with ' + addr[0] + ':' + str(addr[1])
    CONNECTED = True

    #connection established, start receiving Thread (as defines above)
    t = Thread(target=clientthread, args=(conn, pub))
    t.daemon = True
    t.start()


    try:
        while CONNECTED:
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        t.terminate()
        print "Closed TCP-Connection!"
        s.close()



if __name__ == '__main__':
    try:
        tcp_serv()
    except rospy.ROSInterruptException:
        pass
