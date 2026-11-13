#!/usr/bin/python
import socket,struct,time
import sys
import _thread # 因为python3没有thread，只有_thread，如果运行在python2，去掉下划线
import os
class udpTest():
        def __init__(self):
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.send_addr = ('192.168.1.120', 43893)
                # self.send_addr = ('127.0.0.1', 43893)

        def send_data(self,code,value,type):
                data =  struct.pack("<3i", code, value, type)
                self.udp_socket.sendto(data, self.send_addr)
        def repeat_stand_sit(self):
                while True:
                        self.send_data(1, 0, 0)
                        time.sleep(5)
                        self.send_data(1, 0, 0)
                        time.sleep(5)
                        self.send_data(2, 0, 0)
                        time.sleep(2)
        def stand_sit(self):
          self.send_data(0x31010202, 0, 0)
          time.sleep(8)
        def left_shot(self):
          self.send_data(0x21010202, 0, 0)

        def right_shot(self):
          self.send_data(0x21010202, 0, 0)

        def send_heartbeat(self):
          while True:
              time.sleep(0.25)
              try:
                self.send_data(0x21040001, 0, 0)
                print("yun xing zhong")
              except Exception as e:
                  print(e) 

        def say_hi(self):
                self.send_data(1, 0, 0)
                print("start oper")
                try:
                   self.udp_socket.sendto(0x2101020B, self.send_addr)
                except Exception as e:
                   print(e)

        def small_jump(self):
                self.udp_socket.sendto(0x21010206, self.send_addr)

        def half_sit(self):
                self.send_data(2, 0, 0)
                self.udp_socket.sendto(0x21010209, self.send_addr)

        def moon_walk(self):
                self.udp_socket.sendto(0x2101030C, self.send_addr)

        def dance_step_4(self):
                self.send_data(2, 0, 0)
                self.udp_socket.sendto(0x2101030B, self.send_addr)

        def dance_step_3(self):
                self.send_data(2, 0, 0)
                self.udp_socket.sendto(0x2101030A, self.send_addr)

        def dance_step_2(self):
                self.send_data(2, 0, 0)
                self.udp_socket.sendto(0x21010309, self.send_addr)

        def dance_step_1(self):
                print("step 1")
                self.send_data(0x31010204, 0,0)

if __name__ == '__main__':
        u = udpTest()
        #u.circle() 
        #u.straight()
        #u.repeat_circle()
        _thread.start_new_thread(u.send_heartbeat,())
        #u.repeat_stand_sit()
        #u.left_shot()
        #send_heartbeat(u)
        #u.say_hi()
        #u.half_crouch()
        #u.up_head()
        #u.small_jump()
        u.stand_sit()
        u.dance_step_1()
        #u.moon_walk()
        time.sleep(60) #如果不加延时，跳舞将中止
