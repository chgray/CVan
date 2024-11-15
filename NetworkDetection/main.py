from machine import Pin, Timer
import time
import machine, onewire, ds18x20, os
from time import sleep
import network
import ujson
import socket
import time
import ubinascii
import _thread
import os
from Rotary import Rotary
import time
from RotaryIRQ import RotaryIRQ


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


# Setup Rotary Dial
rotaryPower=machine.Pin(21, Pin.OUT)
button=machine.Pin(20, Pin.IN)
rotaryPower.value(1)


def SelectFromList(items):
    scroll = 0
    rotaryOld = -1
    buttonOld = -1
    highlightIdx = 0
    ret = -1
    rotary = RotaryIRQ(pin_num_clk=18, 
              pin_num_dt=19, 
              min_val=0, 
              max_val=len(items), 
              reverse=True, 
              range_mode=RotaryIRQ.RANGE_BOUNDED)
    
    while True:
        rotaryNew = rotary.value()
        buttonNew = button.value()
        
        if rotaryOld == rotaryNew and buttonOld == buttonNew:
            continue
        
        if 0 == buttonNew:
            print("RETURNING %d" % rotary.value())
            ret = rotary.value()
            break
        
        rotaryOld = rotaryNew
        buttonOld = buttonNew
        highlightIdx = rotary.value()
        
        print ("Different (%d, %d)" % (rotaryNew, buttonNew))
        
        scroll = highlightIdx
        
        print ("Scroll : %d" % scroll)
        
        oled.fill(0)
        y = 0
        for idx in range(scroll, min(scroll + 4, len(items)), 1):
                       
            if highlightIdx == idx:
                text = (">%s" % items[idx])                
            else:
                text = (" %s" % items[idx])               
        
            print(text)
            oled.text(text, 0, y)
            y += 15
        oled.show()   
        
        
    start = time.ticks_ms()
    while 0 == button.value():
        if start + 1000 > time.ticks_ms():
            continue
        print("Waiting for button to go up")
        updateLEDScreen("INFO", "Release Button")

    return ret


led_row_3 = ""
led_row_4 = ""
wifi_confidence = 0

def updateLEDScreen(msg, msg2):
    global led_row_3
    global led_row_4
    global wifi_confidence
    oled.fill(0) # Black
    oled.text(msg, 0, 0)   
    oled.text(msg2, 0, 15)   
    oled.text(led_row_3, 0, 30) 
    oled.text(led_row_4, 0, 45)    
    oled.rect(0,50,WIDTH,12, 1)
    
    #p/100 = i/o
    print("Wifi Confidence %d" % wifi_confidence)
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
    #print("Toggle - WD")
    #fixedLed.toggle()
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



def findIdexInArray(list, item):    
    for idx in range(0, len(list), 1):
        if list[idx] == item:
            return idx

    return -1

def DisplayError(msg, msg2, msg3):
    global led_row_3
    global led_row_4
    global button
    led_row_3 = "press key"
    updateLEDScreen(msg, msg2)
    
    while 1 == button.value():
        sleep(0.001)
        
    print("ERROR finished")

def FindWifi():
    wifiIdx = 0
    wifiSSIDs = []
    wifiSSIDs.append("<rescan>")
    
    while 0 == wifiIdx:  
        scanlist = wlan.scan()
        updateLEDScreen("Booting", "Got WiFiScan")

        print("Got Scanlist")
        for result in scanlist:
            ssid, bssid, channel, RSSI, authmode, hidden = result
            
            b = ssid.decode()  
            
            if 0 == len(b):
                print("Skipping %s" % b)
                continue      
            
            if -1 != findIdexInArray(wifiSSIDs, b):
                print("ALREADY KNOW %s" % b)
                continue
            
            print("     %s == %s,  authmode=%d" % (b, ubinascii.hexlify(bssid), authmode))
            wifiSSIDs.append(b)
                        
        hi = SelectFromList(wifiSSIDs)
        wifiIdx = hi
        
        print("SELECTED %d" % hi)
            

    ssid = wifiSSIDs[wifiIdx]
    print("Scan Complete")
    return ssid



def SetPWD(config):
        
    if not ("SelectedNetwork" in config):
        DisplayError("NO Selected Network", "", "")
        return
    ssid = config["SelectedNetwork"]
    
    print(config["pwds"])
    
    if (ssid in config["pwds"]):
        pwd = list(config["pwds"][ssid])
        print("We have pwd")
    else:
        pwd = list()    
        print("NO PWD for %s" % ssid)
    

    #nl = list(pwd)
    #nl[0] = 'x'
    #pwd = "".join(nl)
    
    finished = False
    
    while not finished:
        
        letters = []
        for i in range(0, len(pwd), 1):
            l = pwd[i]
            letters.append(l)
        letters.append("NEW CHARACTER")
        letters.append("DEL CHARACTER")
        letters.append("SAVE")
               
        charIndex = SelectFromList(letters)
        print("Selected Idx : %d" % charIndex)
        
        
        if len(pwd) == charIndex:
            print("NEW CHAR")
            pwd.append('*')
            
        elif len(pwd)+1 == charIndex:
            print("DELCHAR")
            if len(pwd) >= 1:
                del pwd[len(pwd)-1]
            config["pwds"][ssid] = "".join(pwd)
            SaveConfigFile("WiFi.config", config)
            return
            
        elif len(pwd)+2 == charIndex:
            print("SAVE")
            config["pwds"][ssid] = "".join(pwd)
            SaveConfigFile("WiFi.config", config)
            return
        
        letters = []
        letters.append("BACK")
                
        for i in range(0, 26, 1):
            l = chr(ord('A')+i)
            letters.append(l)
            
        for i in range(0, 10, 1):
            l = chr(ord('0')+i)
            
            letters.append(l)
        for i in range(0, 26, 1):
            l = chr(ord('a')+i)
            letters.append(l)
        
        idx = SelectFromList(letters)
        
        
        if 0 == idx:
            print("Back")
        else:
            pwd[charIndex] = letters[idx][0]
        
    return ssid




def TestNetwork(config):
    global led_row_3
    global led_row_4
    global wifi_confidence
    global button
    
    if not ("SelectedNetwork" in config):
        DisplayError("NO Selected Network", "", "")
        return
    
    ssid = config["SelectedNetwork"]
    
    while True:
        print("Looking up : %s" % ssid)
        
        if not (ssid in config["pwds"]):
            DisplayError("NO PASSWORD", ssid, "")
            return
        pwd = config["pwds"][ssid]
        
        #pokeWatchDog()
        updateLEDScreen("Booting", "NET:%s" % ssid);
        wlan.connect(ssid, pwd)

        updateLEDScreen("Connecting!", "NET:%s" % ssid);
        
        while True:
            print('waiting for connection...  Status=%d, connected=%d' % (wlan.status(), wlan.isconnected()))
            if wlan.status() < 0 or wlan.status() >= 3:
                updateLEDScreen("Connected!", "NET:%s" % ssid);
                break
            #pokeWatchDog()
            time.sleep(1)

        if wlan.isconnected():
            status = wlan.ifconfig()
        else:
            print('network connection failed')
            DisplayError("Cant Connect", ("Error:%d" % wlan.status()), ssid )
            return


        print(status)
        print("--- %s" % status[0]) 
        ipAddress = status[0]  
        led_row_3 = ipAddress 
    
        
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
            
            
            if 0 == button.value():
                return
            
            wifi_confidence = (int)((total / (packets * len(goodMsgs))) * 100)
            #updateLEDScreen("CONFIDENCE", ("%s" % wifi_confidence))
            updateLEDScreen(ssid, ("T:%d  R:%d" % (packets * len(goodMsgs), total)))
        #print("%d --- %d" % trans, recv)
    
    
def fileExists(path):
    try:
        os.stat(path)
        return True
    except OSError:
            return False
    
def LoadConfigFile(fileName):   
    if fileExists(fileName):
        print("Config Exists")
        with open(fileName) as configFile:
            value = configFile.readlines()[0]
            print(value)    
            return ujson.loads(value)
    else:
        print("No Config File; assuming defaults")
        return {}
    
def SaveConfigFile(fileName, config):
    with open(fileName,'w') as f:
        f.write(ujson.dumps(config))
    
    
def MainMenu(config):
    start = time.ticks_ms()
    while 0 == button.value():
        if start + 1000 > time.ticks_ms():
            continue
        print("Waiting for button to go up")
        updateLEDScreen("INFO", "Release Button")
               
    menu = []
    menu.append("Test Network")    
    menu.append("Select WiFi")
    menuIdx = SelectFromList(menu)
    
    print("MainMenu Choice: %d" % menuIdx)
       
    if 0 == menuIdx:
        TestNetwork(config)
    elif 1 == menuIdx:
        ssid = FindWifi()
        print("Selected %s" % ssid)
                
        config["SelectedNetwork"] = ssid
        SaveConfigFile("WiFi.config", config);
        
        SetPWD(config)
        return



config = LoadConfigFile("WiFi.config")
print("READ AS JSON %s" % ujson.dumps(config))

print("Modified AS JSON %s" % ujson.dumps(config))
SaveConfigFile("WiFi.config", config)
x = ujson.loads(ujson.dumps(config))
print (x)

TestNetwork(config)

while True:
    MainMenu(config)

