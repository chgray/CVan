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

    print("Got Scanlist")
    for result in scanlist:
        ssid, bssid, channel, RSSI, authmode, hidden = result
        
        b = ssid.decode()
        #= bytearray(ssid, encoding="utf-8")
        
        print("     %s == %s,  authmode=%d" % (b, ubinascii.hexlify(bssid), authmode))
        
        if b == 'Hello':
            print("Hello!")
            m_ssid = "Hello"
            

    print("Scan Complete")

    #pokeWatchDog()
    wlan.connect(m_ssid, m_password)

    while True:
        print('waiting for connection...  Status=%d, connected=%d' % (wlan.status(), wlan.isconnected()))
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        #pokeWatchDog()
        time.sleep(1)

    if wlan.isconnected():
        print('connected')
        status = wlan.ifconfig()
    else:
        print('network connection failed')
        continue

    #addr = socket.getaddrinfo('0.0.0.0', 25)[0][-1]

    #pokeWatchDog()
    #s = socket.socket()
    #s.bind(addr)
    #s.listen(1)

    #print('listening on', addr)

    ping('8.8.8.8', count=4000, timeout=5000, interval=50, quiet=False, size=64)
