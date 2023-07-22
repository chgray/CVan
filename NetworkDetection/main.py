from machine import Pin, Timer
import time
import machine, onewire, ds18x20, os
from time import sleep
import network
import socket
import time
import ubinascii
import _thread
import os
#import ipaddress
#import wifi
#import socketpoo


# https://gist.github.com/shawwwn/91cc8979e33e82af6d99ec34c38195fb

from time import sleep
from machine import Pin
from machine import WDT

import machine
import time
from time import sleep


 # Display Image & text on I2C driven ssd1306 OLED display 
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import framebuf
import math
import utime
WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height

# Explicit Method
sda=machine.Pin(4)
scl=machine.Pin(5)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
#  print(i2c.scan())
from ssd1306 import SSD1306_I2C
oled = SSD1306_I2C(128, 64, i2c)


led_row_3 = ""
led_row_4 = ""
wifi_confidence = 50

def updateLEDScreen(msg, msg2):
    oled.fill(0) # Black
    oled.text(msg, 0, 0)   
    oled.text(msg2, 0, 15)   
    oled.text(led_row_3, 0, 30) 
    oled.text(led_row_4, 0, 45)    
    oled.rect(0,50,WIDTH,12, 1)
    
    #p/100 = i/o
    w = (int)(wifi_confidence/100 * (WIDTH-2))
    oled.fill_rect(1, 50, w, 12, 1)
    
    #oled.fill_rect(40,40,20,10,1)
    oled.show()
        

updateLEDScreen("Booting", "Init")


fixedLed = Pin("LED", Pin.OUT)
wdt = WDT(timeout=8000) #timeout is in ms
timer = Timer()

def pokeWatchDog():
    global wdt
    global fixedLed
    
    #led = Pin("LED", Pin.OUT)
    print("Toggle - WD")
    fixedLed.toggle()
    wdt.feed() #resets countdown

def pokeWatchDogTimer(t):
    #print("Watchdog Timer")
    pokeWatchDog()

timer.init(mode=Timer.PERIODIC, period=1000, callback=pokeWatchDogTimer)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
updateLEDScreen("Booting", "Wifi on")







# ---------------------------


# µPing (MicroPing) for MicroPython
# copyright (c) 2018 Shawwwn <shawwwn1@gmail.com>
# License: MIT

# Internet Checksum Algorithm
# Author: Olav Morken
# https://github.com/olavmrk/python-ping/blob/master/ping.py
# @data: bytes
def checksum(data):
    if len(data) & 0x1: # Odd number of bytes
        data += b'\0'
    cs = 0
    for pos in range(0, len(data), 2):
        b1 = data[pos]
        b2 = data[pos + 1]
        cs += (b1 << 8) + b2
    while cs >= 0x10000:
        cs = (cs & 0xffff) + (cs >> 16)
    cs = ~cs & 0xffff
    return cs

def ping(host, count=4, timeout=5000, interval=10, quiet=False, size=64):
    import utime
    import uselect
    import uctypes
    import usocket
    import ustruct
    import urandom
    
    print("Ping : %s", host);
    global fixedLed

    # prepare packet
    assert size >= 16, "pkt size too small"
    pkt = b'Q'*size
    pkt_desc = {
        "type": uctypes.UINT8 | 0,
        "code": uctypes.UINT8 | 1,
        "checksum": uctypes.UINT16 | 2,
        "id": uctypes.UINT16 | 4,
        "seq": uctypes.INT16 | 6,
        "timestamp": uctypes.UINT64 | 8,
    } # packet header descriptor
    h = uctypes.struct(uctypes.addressof(pkt), pkt_desc, uctypes.BIG_ENDIAN)
    h.type = 8 # ICMP_ECHO_REQUEST
    h.code = 0
    h.checksum = 0
    h.id = urandom.randint(0, 65535)
    h.seq = 1

    # init socket
    sock = usocket.socket(usocket.AF_INET, usocket.SOCK_RAW, 1)
    sock.setblocking(0)
    sock.settimeout(timeout/1000)
    addr = usocket.getaddrinfo(host, 1)[0][-1][0] # ip address
    sock.connect((addr, 1))
    not quiet and print("PING %s (%s): %u data bytes" % (host, addr, len(pkt)))

    seqs = list(range(1, count+1)) # [1,2,...,count]
    c = 1
    t = 0
    n_trans = 0
    n_recv = 0
    finish = False
    while t < timeout:
        if t==interval and c<=count:
            # send packet
            h.checksum = 0
            h.seq = c
            h.timestamp = utime.ticks_us()
            h.checksum = checksum(pkt)
            if sock.send(pkt) == size:
                n_trans += 1
                t = 0 # reset timeout
            else:
                seqs.remove(c)
            c += 1
            # print("sent...")

        # recv packet
        while 1:
            # print("...listenting")
            socks, _, _ = uselect.select([sock], [], [], 0)
            if socks:
                resp = socks[0].recv(4096)
                # print("...recv'ed")
                
                resp_mv = memoryview(resp)
                h2 = uctypes.struct(uctypes.addressof(resp_mv[20:]), pkt_desc, uctypes.BIG_ENDIAN)
                # TODO: validate checksum (optional)
                seq = h2.seq
                if h2.type==0 and h2.id==h.id and (seq in seqs): # 0: ICMP_ECHO_REPLY
                    t_elasped = (utime.ticks_us()-h2.timestamp) / 1000
                    ttl = ustruct.unpack('!B', resp_mv[8:9])[0] # time-to-live
                    n_recv += 1
                    fixedLed.toggle()
                    not quiet and print("%u bytes from %s: icmp_seq=%u, ttl=%u, time=%f ms" % (len(resp), addr, seq, ttl, t_elasped))
                    seqs.remove(seq)
                    if len(seqs) == 0:
                        finish = True
                        break
            else:
                break

        if finish:
            break

        utime.sleep_ms(1)
        t += 1

    # close
    sock.close()
    ret = (n_trans, n_recv)
    not quiet and print("%u packets transmitted, %u packets received" % (n_trans, n_recv))
    return (n_trans, n_recv)

# ---------------------------





m_ssid = 'HelloX'
m_password = 'deadbeef01'


while True:
    scanlist = wlan.scan()
    updateLEDScreen("Booting", "Got WiFiScan")

    print("Got Scanlist")
    for result in scanlist:
        ssid, bssid, channel, RSSI, authmode, hidden = result
        
        b = ssid.decode()        
        print("     %s == %s,  authmode=%d" % (b, ubinascii.hexlify(bssid), authmode))
        
        if b == 'Hello':
            print("Hello!")
            m_ssid = "Hello"
            

    print("Scan Complete")

    #pokeWatchDog()
    updateLEDScreen("Booting", "NET:%s" % m_ssid);
    wlan.connect(m_ssid, m_password)

    updateLEDScreen("Connecting!", "NET:%s" % m_ssid);
    
    while True:
        print('waiting for connection...  Status=%d, connected=%d' % (wlan.status(), wlan.isconnected()))
        if wlan.status() < 0 or wlan.status() >= 3:
            updateLEDScreen("Connected!", "NET:%s" % m_ssid);
            break
        #pokeWatchDog()
        time.sleep(1)

    if wlan.isconnected():
        status = wlan.ifconfig()
    else:
        print('network connection failed')
        continue


    print(status)
    print("--- %s" % status[0]) 
    ipAddress = status[0]  
    led_row_3 = ipAddress 
   
    #pokeWatchDog()
    #s = socket.socket()
    #s.bind(addr)
    #s.listen(1)

    #print('listening on', addr)
    
    # return (n_trans, n_recv)
    trans = 5000
    recv = 0
    idx = 0
    goodMsgs = []
    packets = 2
    steps = 20
    
    for idx in range(0, steps, 1):
        print("Setting msg idx %d" % idx)
        goodMsgs.append(0)
        goodMsgs[idx] = 0
            
    while True:
        if False == wlan.isconnected():
            print("NOT onnected")
            break        
       
        ntrans, nrecv = ping('8.8.8.8', count=packets, timeout=500, interval=50, quiet=False, size=64)
        trans += ntrans
        recv += nrecv
        
        idx += 1
        if idx >= len(goodMsgs):
            idx = 0
        
        goodMsgs[idx] = nrecv
        total = 0
        for i in range(0, len(goodMsgs), 1):
            total += goodMsgs[i]
        
        wifi_confidence = (int)((total / (packets * len(goodMsgs))) * 100)
        #updateLEDScreen("CONFIDENCE", ("%s" % wifi_confidence))
        updateLEDScreen(m_ssid, ("T:%d  R:%d" % (packets * len(goodMsgs), total)))
    #print("%d --- %d" % trans, recv)
    