##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2010-2016 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2020 DreamSourceLab <support@dreamsourcelab.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

# TODO: Look into arbitration, collision detection, clock synchronisation, etc.
# TODO: Implement support for inverting SDATA/SCLK levels (0->1 and 1->0).
# TODO: Implement support for detecting various bus errors.

import sigrokdecode as srd

'''
OUTPUT_PYTHON format:

Packet:
[SSC,<Command Frame>, <Data Frame>]

SSC:Sequence Start Condition

<Command Frame>:
 - 'SA' (Slave Address)
 - 'COMMAND' (command Key)
 - 'BC' (Byte Count)
 - 'P' (Parity)
 - 'ADDRESS' (Register Address)
 - 'DATA WRITE0' (Register 0 Write Data)
<Data Frame>:
 - 'ADDRESS' (Register Address)
 - 'P' (Parity)
 - 'BP' (Bus Park)
 - 'DATA READ' (Data, read)
 - 'DATA WRITE' (Data, write)

 <Command Frame> : A Command Frame shall consist of a 4-bit Slave address field, an 8-bit command payload field, and a single parity bit.

 <Data Frame> : A Data or Address Frame shall consist of eight data bits or eight address bits, respectively, and a single parity bit. 
'''
# cmd: [annotation-type-index, long annotation, short annotation]
proto = {
    'SSC':             [0, 'Sequence Start Condition',       'SSC'],
    'SA':              [1, 'Slave Address',                   'SA'],
    'ERW':             [2, 'Extended Register Write',        'ERW'],
    'ERR':             [3, 'Extended Register Read',         'ERR'],
    'ERWL':            [4, 'Extended Register Write Long',  'ERWL'],
    'ERRL':            [5, 'Extended Register Read Long',   'ERRL'],
    'RW':              [6, 'Register Write',                  'RW'],
    'RR':              [7, 'Register Read',                   'RR'],
    'R0W':             [8, 'Register 0 Write',               'R0W'],
    'BC':              [9, 'Byte',                            'BC'],
    'P':               [10, 'Parity',                          'P'],
    'ADDRESS':         [11, 'Address',                         'A'],
    'BP':              [12, 'Bus Pack',                       'BP'],
    'DATA':            [13, 'Data ',                        'DATA'],
    'CMD_WARNINGS':    [14, 'Command Warnings',         'CMD_WARN'],
    'BIC':             [15, 'Bus Idle Condition ',           'BIC'],
    'SSC_WARNINGS':    [16, 'SSC Warnings',             'SCC_WARN'],
    'IJE':             [17, 'Illegal Jump Edge',        'IJE_WAEN'],
}

class Decoder(srd.Decoder):
    api_version = 3
    id = 'rffe'
    name = 'RFFE'
    longname = 'RF Front-End Control Interface'
    desc = 'Two-wire, single-master, serial bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['rffe']
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'sclk', 'type': 8, 'name': 'SCLK', 'desc': 'Serial clock line'},
        {'id': 'sdata', 'type': 108, 'name': 'SDATA', 'desc': 'Serial data line'},
    )
    options = (
        {'id': 'address_format', 'desc': 'Displayed slave address format',
            'default': 'shifted', 'values': ('shifted', 'unshifted')},
    )
    annotations = (
        ('7', 'ssc', 'Sequence Start Condition'),
        ('6', 'sa', 'Slave Address'),
        ('1', 'erw', 'Extended register write'),
        ('5', 'err', 'Extended register read'),
        ('0', 'erwl', 'Extended register write long'),
        ('112', 'errl', 'Extended register read long'),
        ('111', 'rw', 'Register write'),
        ('110', 'rr', 'Register read'),
        ('109', 'r0w', 'Register 0 write'),
        ('108', 'bc', 'Byte'),
        ('80', 'p', 'Parity'),
        ('75', 'address', 'Address'),
        ('70', 'bp', 'Bus pack'),
        ('65', 'data', 'DATA'),
        ('1000', 'Command warnings', 'Command warnings'),
        ('50', 'Bus Idle Condition ', 'Bus Idle Condition '),
        ('1000', 'SSC warnings', 'SSC warnings'),
        ('1000', 'Illegal Jump Edge', 'IJE_WAEN'),
    )
    annotation_rows = (
        ('command-data', 'Command/Data', (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 , 11, 12, 13 ,15)),
        ('warnings', 'Warnings', (14,16,17,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.ss = self.es = -1
        self.bitcount = 0
        self.databyte = 0
        self.state = 'FIND BUS_PARK'
        self.extended = -1
        self.cmdkey = 'NULL'
        self.BC = 0
        self.Pcount = 0
        self.BPcount = 0
        self.ADDcount = 0
        self.BPss = 0
        self.SSCs = 0
        self.Pes = 0

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        self.put(self.ss, self.es, self.out_ann, data)
    
    def handle_BP(self,cmd,state):

        self.ss = self.Pes
        self.es = self.samplenum        
        self.putx([proto[cmd][0], proto[cmd][1:]])
        self.state = state

    def handle(self,sclk,sdata,cmd,state,key):
        
        self.databyte <<= 1
        self.databyte |= sdata

        if self.bitcount == 0:
            self.DATAss = self.samplenum
        
        if self.bitcount < key:
            while True :
                self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                if (self.matched & (0b1 << 0)):
                    self.ss = self.samplenum
                    self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                    if (self.matched & (0b1 << 0)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                    if (self.matched & (0b1 << 1)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                        break
                if (self.matched & (0b1 << 1)):
                    break
            self.bitcount += 1
            return
        
        d = self.databyte
        
        while True :
            self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
            if (self.matched & (0b1 << 0)):
                self.ss = self.samplenum
                self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                if (self.matched & (0b1 << 0)):
                    self.es = self.samplenum
                    self.putx([proto['IJE'][0], proto['IJE'][1:]])
                if (self.matched & (0b1 << 1)):
                    self.es = self.samplenum
                    self.putx([proto['IJE'][0], proto['IJE'][1:]])
                    break
            if (self.matched & (0b1 << 1)):
                break
        self.ss = self.DATAss
        self.es = self.samplenum
        if cmd == 'P':
            self.Pes = self.samplenum
        self.putx([proto[cmd][0], ['%s[%d:0]: %02X' % (proto[cmd][1],key,d),
                   '%s[%d:0]: %02X' % (proto[cmd][2],key,d), '%02X' % d]])
        self.bitcount = self.databyte = 0
        self.state = state

    def handle_CMD(self,sclk,sdata):
        
        if self.bitcount == 0:
            self.DATAss = self.samplenum
            if sdata :
                self.cmdset('R0W','FIND DATA')
                return

        if self.bitcount == 1:
            if sdata :
                self.extended = 0
            else :
                self.extended = 1    

        if self.bitcount == 2:
            if ~self.extended :
                if sdata :
                    self.cmdset('RR','FIND ADDRESS')
                    return
                else :
                    self.cmdset('RW','FIND ADDRESS')
                    return
            
        if self.bitcount == 3:
            if ~sdata :
                if self.extended :
                    self.cmdset('ERR','FIND BTEY_COUNT')
                    return
                else :
                    self.cmdset('ERW','FIND BTEY_COUNT')
                    return
            elif self.extended :
                self.es = self.samplenum
                self.putx([proto['CMD_WARNINGS'][0], proto['CMD_WARNINGS'][1:]])
                self.init()
                return

        if self.bitcount == 4:
            if sdata :
                self.cmdset('ERRL','FIND BTEY_COUNT')
                return
            else :
                self.cmdset('ERWL','FIND BTEY_COUNT')
                return

        if self.bitcount <4:
            while True :
                (sclk, sdata) = self.wait([{0: 'l', 1: 'e'},{0: 'h'}])
                if (self.matched & (0b1 << 0)):
                    self.ss = self.samplenum
                    (sclk, sdata) = self.wait([{0: 'l', 1: 'e'},{0: 'h'}])
                    if (self.matched & (0b1 << 0)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                    if (self.matched & (0b1 << 1)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                        break
                if (self.matched & (0b1 << 1)):
                    break
            self.bitcount += 1

    def cmdset(self,cmd,state):
        while True :
            self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
            if (self.matched & (0b1 << 0)):
                self.ss = self.samplenum
                self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                if (self.matched & (0b1 << 0)):
                    self.es = self.samplenum
                    self.putx([proto['IJE'][0], proto['IJE'][1:]])
                if (self.matched & (0b1 << 1)):
                    self.es = self.samplenum
                    self.putx([proto['IJE'][0], proto['IJE'][1:]])
                    break
            if (self.matched & (0b1 << 1)):
                break

        self.ss = self.DATAss
        self.es = self.samplenum
        self.putx([proto[cmd][0], proto[cmd][1:]])
        self.state = state
        self.bitcount = 0
        self.extended = -1
        self.cmdkey = cmd

    def initBP(self,sclk,sdata,state,key):
        self.handle_BP('BP',state)
        self.state = state  
        if key :
            self.init()
           
    def init(self):
        self.cmdkey = 'NULL'
        self.ADDcount = 0
        self.Pcount = 0
        self.BPcount = 0
        self.BC = 0
        self.bitcount = 0
        self.Pes = 0
        if self.state == 'FIND COMMAND':
            self.state = 'FIND BUS_PARK'
        else:
            self.state = 'FIND SSC'

    def decode(self):
        while True:
            if self.state == 'FIND SSC':       
                self.wait([{0: 'h'},{0: 'l', 1: 'f'}])
                if (self.matched & (0b1 << 0)):
                    self.ss = self.es = self.samplenum
                    self.putx([proto['SSC_WARNINGS'][0], proto['SSC_WARNINGS'][1:]])
                    self.state = 'FIND BUS_PARK'
                if (self.matched & (0b1 << 1)):
                    self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                    if (self.matched & (0b1 << 0)):
                        self.ss = self.samplenum
                        self.wait([{0: 'l', 1: 'e'},{0: 'r'}])
                        if (self.matched & (0b1 << 0)):
                            self.es = self.samplenum
                            self.putx([proto['IJE'][0], proto['IJE'][1:]])
                        if (self.matched & (0b1 << 1)):
                            self.es = self.samplenum
                            self.putx([proto['IJE'][0], proto['IJE'][1:]])
                            self.ss,self.es = self.BPss,self.samplenum
                            self.putx([proto['SSC'][0], proto['SSC'][1:]])
                            self.state = 'FIND SLAVE ADDRESS'
                            continue
                    if (self.matched & (0b1 << 1)):
                        self.ss,self.es = self.BPss,self.samplenum
                        self.putx([proto['SSC'][0], proto['SSC'][1:]])
                        self.state = 'FIND SLAVE ADDRESS'
                    
            elif self.state == 'FIND SLAVE ADDRESS':
                (sclk, sdata) = self.wait({0: 'h'})
                self.handle(sclk,sdata,'SA','FIND COMMAND',3)
            
            elif self.state == 'FIND COMMAND':
                (sclk, sdata) = self.wait({0: 'h'})
                self.handle_CMD(sclk,sdata)
                
            elif  self.state == 'FIND BTEY_COUNT':
                (sclk, sdata) = self.wait({0: 'h'})
                if self.cmdkey == 'ERW' or self.cmdkey == 'ERR' :                   
                    self.handle(sclk,sdata,'BC','FIND PARITY',3)
                else :                   
                    self.handle(sclk,sdata,'BC','FIND PARITY',2)

            elif self.state == 'FIND ADDRESS':
                (sclk, sdata) = self.wait({0: 'h'})
                if self.cmdkey == 'RW' or self.cmdkey == 'RR' :                  
                    self.handle(sclk,sdata,'ADDRESS','FIND PARITY',4)
                else :                   
                    self.handle(sclk,sdata,'ADDRESS','FIND PARITY',7)

            elif self.state == 'FIND DATA':
                (sclk, sdata) = self.wait({0: 'h'})
                if self.cmdkey == 'R0W' :                   
                    self.handle(sclk,sdata,'DATA','FIND PARITY',6)
                elif self.cmdkey == 'RW' or self.cmdkey == 'RR' :                  
                    self.handle(sclk,sdata,'DATA','FIND PARITY',7)
                else :                   
                    self.handle(sclk,sdata,'DATA','FIND PARITY',(self.BC*8-1))

            elif self.state == 'FIND PARITY':
                (sclk, sdata) = self.wait({0: 'h'})
                self.Pcount += 1
                self.handle(sclk,sdata,'P','NULL',0)
                if self.cmdkey == 'R0W' :
                    self.state = 'FIND BUS_PARK'

                elif self.cmdkey == 'ERW' :
                    if self.Pcount == 1 :
                        self.ADDcount,self.state = 1,'FIND ADDRESS'  
                    elif self.Pcount == 2 :
                        self.state = 'FIND DATA'

                elif self.cmdkey == 'ERR' :
                    if self.Pcount == 1 :
                        self.ADDcount,self.state = 1,'FIND ADDRESS' 
                    elif self.Pcount == 2 :
                        self.BPcount,self.state = 1,'FIND BUS_PARK'

                elif self.cmdkey == 'ERWL' :
                    if self.Pcount == 1 :
                        self.ADDcount,self.state = 2,'FIND ADDRESS' 
                    elif self.Pcount == 2 :    
                        self.ADDcount,self.state = 1,'FIND ADDRESS' 
                    elif self.Pcount == 3 :
                        self.state = 'FIND DATA'
                    
                elif self.cmdkey == 'ERRL' :
                    if self.Pcount == 1 :
                        self.ADDcount,self.state = 2,'FIND ADDRESS'
                    elif self.Pcount == 2 :
                        self.ADDcount,self.state = 1,'FIND ADDRESS'
                    elif self.Pcount == 3 :
                        self.BPcount,self.state = 1,'FIND BUS_PARK'

                elif self.cmdkey == 'RW' :
                    if self.Pcount == 1 :    
                        self.state = 'FIND DATA'
                    elif self.Pcount == 2 :   
                        self.BPcount,self.state = 1,'FIND BUS_PARK'

                elif self.cmdkey == 'RR' :
                    self.state = 'FIND BUS_PARK'
                    self.BPcount = 1 if (self.Pcount == 1) else 2 

            elif self.state == 'FIND BUS_PARK':
                (sclk, sdata) = self.wait([{0: 'l', 1: 'r'},{0: 'l', 1: 'l'}])
                if (self.matched & (0b1 << 0)):
                    self.ss = self.samplenum
                    self.wait([{0: 'l', 1: 'e'},{0: 'l', 1: 'l'}])
                    if (self.matched & (0b1 << 0)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                    if (self.matched & (0b1 << 1)):
                        self.es = self.samplenum
                        self.putx([proto['IJE'][0], proto['IJE'][1:]])
                if (self.matched & (0b1 << 1)):
                    self.BPss = self.samplenum
                    if self.cmdkey == 'NULL':
                        self.wait([{0 : 'r', 1 : 'l'},{0: 'l', 1: 'r'}])
                        if (self.matched & (0b1 << 0)):
                            self.ss = self.samplenum
                            self.wait([{0 : 'r', 1 : 'l'},{0: 'f', 1: 'l'},{0: 'l', 1: 'r'}])
                            if (self.matched & (0b1 << 0)):
                                self.es =self.samplenum
                                self.putx([proto['SSC_WARNINGS'][0], proto['SSC_WARNINGS'][1:]])
                                self.state ='FIND BUS_PARK'
                            if (self.matched & (0b1 << 1)):
                                self.BPss = self.samplenum
                            if (self.matched & (0b1 << 2)):
                                self.state ='FIND SSC' 
                                continue
                        if (self.matched & (0b1 << 1)):
                            # self.SSCs = self.samplenum
                            self.state ='FIND SSC'   

                    elif self.cmdkey == 'ERR' or self.cmdkey == 'ERRL' or self.cmdkey == 'RR':
                        key = 0 if (self.BPcount == 1) else 1 
                        self.initBP(sclk,sdata,'FIND DATA',key)
                    else :
                        self.initBP(sclk,sdata,'FIND SSC',1)

            


                
                
        