import spidev
import time
import RPi.GPIO as GPIO

class MCP2515(object):
    
    # Pin Definition
    MCP2515_RESET   = 23
    MCP2515_INT     = 24

    # Error Messages
    MYCAN_ERR_INIT          = "Unable to init MCP2515"
    MYCAN_ERR_FRAME_TYPE    = "Unknown frame type - unable to send msg"

    # MCP2515 Registers
    MCP2515_TX_STD_FRAME        = 0x00
    MCP2515_TX_REMOTE_FRAME     = 0x01

    # CAN configuration registers

    MCP2515_BFPCTRL         = 0x0C
    MCP2515_TXRTSCTRL       = 0x0D
    MCP2515_CANSTAT         = 0x0E
    MCP2515_CANCTRL         = 0x0F

    MCP2515_TEC             = 0x1C
    MCP2515_REC             = 0x1D

    MCP2515_CNF3       = 0x28
    MCP2515_CNF2       = 0x29
    MCP2515_CNF1       = 0x2A

    MCP2515_CANINTE    = 0x2B
    MCP2515_CANINTF    = 0x2C
    MCP2515_EFLG       = 0x2D

    # CAN Receive Mask/Filter registers

    MCP2515_RXM0SIDH   = 0x20
    MCP2515_RXM0SIDL   = 0x21
    MCP2515_RXM0EID8   = 0x22
    MCP2515_RXM0EID0   = 0x23

    MCP2515_RXM1SIDH   = 0x24
    MCP2515_RXM1SIDL   = 0x25
    MCP2515_RXM1EID8   = 0x26
    MCP2515_RXM1EID0   = 0x27

    MCP2515_RXF0SIDH   = 0x00
    MCP2515_RXF0SIDL   = 0x01
    MCP2515_RXF0EID8   = 0x02
    MCP2515_RXF0EID0   = 0x03

    MCP2515_RXF1SIDH   = 0x04
    MCP2515_RXF1SIDL   = 0x05
    MCP2515_RXF1EID8   = 0x06
    MCP2515_RXF1EID0   = 0x07

    MCP2515_RXF2SIDH   = 0x08
    MCP2515_RXF2SIDL   = 0x09
    MCP2515_RXF2EID8   = 0x0A
    MCP2515_RXF2EID0   = 0x0B

    MCP2515_RXF3SIDH   = 0x10
    MCP2515_RXF3SIDL   = 0x11
    MCP2515_RXF3EID8   = 0x12
    MCP2515_RXF3EID0   = 0x13

    MCP2515_RXF4SIDH   = 0x14
    MCP2515_RXF4SIDL   = 0x15
    MCP2515_RXF4EID8   = 0x16
    MCP2515_RXF4EID0   = 0x17

    MCP2515_RXF5SIDH   = 0x18
    MCP2515_RXF5SIDL   = 0x19
    MCP2515_RXF5EID8   = 0x1A
    MCP2515_RXF5EID0   = 0x1B

    # CAN Transmit Control/Header/Data registers

    MCP2515_TXB0CTRL   = 0x30
    MCP2515_TXB0SIDH   = 0x31
    MCP2515_TXB0SIDL   = 0x32
    MCP2515_TXB0EID8   = 0x33
    MCP2515_TXB0EID0   = 0x34
    MCP2515_TXB0DLC    = 0x35
    MCP2515_TXB0D0     = 0x36
    MCP2515_TXB0D1     = 0x37
    MCP2515_TXB0D2     = 0x38
    MCP2515_TXB0D3     = 0x39
    MCP2515_TXB0D4     = 0x3A
    MCP2515_TXB0D5     = 0x3B
    MCP2515_TXB0D6     = 0x3C
    MCP2515_TXB0D7     = 0x3D

    MCP2515_TXB1CTRL   = 0x40
    MCP2515_TXB1SIDH   = 0x41
    MCP2515_TXB1SIDL   = 0x42
    MCP2515_TXB1EID8   = 0x43
    MCP2515_TXB1EID0   = 0x44
    MCP2515_TXB1DLC    = 0x45
    MCP2515_TXB1D0     = 0x46
    MCP2515_TXB1D1     = 0x47
    MCP2515_TXB1D2     = 0x48
    MCP2515_TXB1D3     = 0x49
    MCP2515_TXB1D4     = 0x4A
    MCP2515_TXB1D5     = 0x4B
    MCP2515_TXB1D6     = 0x4C
    MCP2515_TXB1D7     = 0x4D

    MCP2515_TXB2CTRL   = 0x50
    MCP2515_TXB2SIDH   = 0x51
    MCP2515_TXB2SIDL   = 0x52
    MCP2515_TXB2EID8   = 0x53
    MCP2515_TXB2EID0   = 0x54
    MCP2515_TXB2DLC    = 0x55
    MCP2515_TXB2D0     = 0x56
    MCP2515_TXB2D1     = 0x57
    MCP2515_TXB2D2     = 0x58
    MCP2515_TXB2D3     = 0x59
    MCP2515_TXB2D4     = 0x5A
    MCP2515_TXB2D5     = 0x5B
    MCP2515_TXB2D6     = 0x5C
    MCP2515_TXB2D7     = 0x5D

    # CAN Receive Control/Header/Data registers

    MCP2515_RXB0CTRL   = 0x60
    MCP2515_RXB0SIDH   = 0x61
    MCP2515_RXB0SIDL   = 0x62
    MCP2515_RXB0EID8   = 0x63
    MCP2515_RXB0EID0   = 0x64
    MCP2515_RXB0DLC    = 0x65
    MCP2515_RXB0D0     = 0x66
    MCP2515_RXB0D1     = 0x67
    MCP2515_RXB0D2     = 0x68
    MCP2515_RXB0D3     = 0x69
    MCP2515_RXB0D4     = 0x6A
    MCP2515_RXB0D5     = 0x6B
    MCP2515_RXB0D6     = 0x6C
    MCP2515_RXB0D7     = 0x6D

    MCP2515_RXB1CTRL   = 0x70
    MCP2515_RXB1SIDH   = 0x71
    MCP2515_RXB1SIDL   = 0x72
    MCP2515_RXB1EID8   = 0x73
    MCP2515_RXB1EID0   = 0x74
    MCP2515_RXB1DLC    = 0x75
    MCP2515_RXB1D0     = 0x76
    MCP2515_RXB1D1     = 0x77
    MCP2515_RXB1D2     = 0x78
    MCP2515_RXB1D3     = 0x79
    MCP2515_RXB1D4     = 0x7A
    MCP2515_RXB1D5     = 0x7B
    MCP2515_RXB1D6     = 0x7C
    MCP2515_RXB1D7     = 0x7D

    MCP2515_CMD_RESET      = 0xC0
    MCP2515_CMD_WRITE      = 0x02
    MCP2515_CMD_READ       = 0x03
    MCP2515_CMD_RTS        = 0x80
    MCP2515_CMD_BITMOD     = 0x05
    MCP2515_CMD_STATUS     = 0xA0
    MCP2515_CMD_RXSTATUS   = 0xB0
    
    transmit_ready = True

    def __init__(self):
        self._spi = spidev.SpiDev()
        self._spi.open(0,1)
        self._spi.max_speed_hz = 500000
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.MCP2515_INT, GPIO.IN)
        GPIO.setup(self.MCP2515_RESET, GPIO.OUT)
        GPIO.output(self.MCP2515_RESET, GPIO.HIGH)
        GPIO.output(self.MCP2515_RESET, GPIO.LOW)
        GPIO.output(self.MCP2515_RESET, GPIO.HIGH)
        
        self.DoInit()

    def Is_transmit_ready(self):
        return (self.transmit_ready)

    def _writeCommand(self, theCommand):
        ToSPI = [theCommand] 
        self._spi.xfer2(ToSPI)

    def _readCommand(self, theCommand):
        ToSPI = [theCommand, 0x00, 0x00] 
        FromSPI = self._spi.xfer2(ToSPI)
        x = FromSPI[1]
        return (x) 
        
    def _writeReg(self, theAddress, theData):
        ToSPI = [self.MCP2515_CMD_WRITE, theAddress, theData]
        self._spi.xfer2(ToSPI)

    def _readReg(self, theAddress):
        ToSPI = [self.MCP2515_CMD_READ, theAddress, 0x00]
        FromSPI = self._spi.xfer2(ToSPI)
        x = FromSPI[2]
        return (x)   
        
    def DoInit(self):

        # Software Reset
        self._writeCommand(self.MCP2515_CMD_RESET)

        # Set Configuration Mode
        self._writeReg(self.MCP2515_CANCTRL, 0x80)

        """
        Bit Timing Configuration
            125 kHz CAN baud rate with Fosc = 16 MHz

            Fosc        = 16MHz
            BRP         =   7  (divide by 8)
            Sync Seg    = 1TQ
            Prop Seg    = 1TQ
            Phase Seg1  = 3TQ
            Phase Seg2  = 3TQ

            TQ = 2 * (1/Fosc) * (BRP+1) = 1us
            CAN Bit Time = (1+1+3+3) 1 us = 8 us -> 125kHz
            Bus speed = 1/(Total # of TQ) * TQ   

        The CAN Configuration Registers are then assembled as follows:
            CNF1 = SJW1 + (BRP-1) = 0x07
            CNF2 = (BTLMODE_SET + (PHSEG1-1)*8 + (PRSEG-1)) = 0x90
            CNF3 = (SOF_DISABLE + WAKFIL_DISABLE + (PHSEG2-1)) = 0x02
        """
        
        self._writeReg(self.MCP2515_CNF1, 0x07)
        self._writeReg(self.MCP2515_CNF2, 0x90)
        self._writeReg(self.MCP2515_CNF3, 0x02)

        # Configure initialization of message transmission
        # TX0RTS, TX1RTS, TX2RTS not used : B0RTSM, B1RTSM, B2RTSM = 0

        self._writeReg(self.MCP2515_TXRTSCTRL, 0x00) 
    
        # Configure Masks and Filters

        self._writeReg(self.MCP2515_RXM0SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXM0SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXM0EID8, 0x00) 
        self._writeReg(self.MCP2515_RXM0EID0, 0x00)

        self._writeReg(self.MCP2515_RXM1SIDH, 0x00)
        self._writeReg(self.MCP2515_RXM1SIDL, 0x00)
        self._writeReg(self.MCP2515_RXM1EID8, 0x00)
        self._writeReg(self.MCP2515_RXM1EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF0SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF0SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF0EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF0EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF1SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF1SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF1EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF1EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF2SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF2SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF2EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF2EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF3SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF3SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF3EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF3EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF4SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF4SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF4EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF4EID0, 0x00) 

        self._writeReg(self.MCP2515_RXF5SIDH, 0x00) 
        self._writeReg(self.MCP2515_RXF5SIDL, 0x00) 
        self._writeReg(self.MCP2515_RXF5EID8, 0x00) 
        self._writeReg(self.MCP2515_RXF5EID0, 0x00)         

        # Set Normal Operation Mode
        self._writeReg(self.MCP2515_CANCTRL, 0x00)

        # Configura Receive buffer RXB0 et RXB1
        # RXM = 11 : turn mask/filters off
        # BUKT = 1 : enable Rollover

        self._writeReg(self.MCP2515_RXB0CTRL, 0xff)
        self._writeReg(self.MCP2515_RXB1CTRL, 0xff)

        # Clear all interrupt flags
        # Disable all interrupts except
        #   Bit 7 MERRE Message Interrupt Enable
        #   Bit 2 TX0E Transmit Buffer 0 Empty
        #   Bit 1 RX1E Receive  Buffer 1 Full
        #   Bit 0 RX0E Receive  Buffer 0 Full 

        self._writeReg(self.MCP2515_CANINTF, 0x00)
        self._writeReg(self.MCP2515_CANINTE, 0x87)
    
    def DoDebug(self):
        print('CNF1     = {0:2x}'.format(self._readReg(self.MCP2515_CNF1)))
        print('CNF2     = {0:2x}'.format(self._readReg(self.MCP2515_CNF2)))
        print('CNF3     = {0:2x}'.format(self._readReg(self.MCP2515_CNF3)))
        print('CANCTRL  = {0:2x}'.format(self._readReg(self.MCP2515_CANCTRL)))
        print('CANSTAT  = {0:2x}'.format(self._readReg(self.MCP2515_CANSTAT)))
        print('TXB0CTRL = {0:2x}'.format(self._readReg(self.MCP2515_TXB0CTRL)))
        print('TXB1CTRL = {0:2x}'.format(self._readReg(self.MCP2515_TXB1CTRL)))
        print('CANINTE  = {0:2x}'.format(self._readReg(self.MCP2515_CANINTE)))
        print('CANINTF  = {0:2x}'.format(self._readReg(self.MCP2515_CANINTF)))
        print('EFLG     = {0:2x}'.format(self._readReg(self.MCP2515_EFLG)))
        print('TEC      = {0:2x}'.format(self._readReg(self.MCP2515_TEC)))
        print('REC      = {0:2x}'.format(self._readReg(self.MCP2515_REC)))
        
        print('Status   = {0:2x}'.format(self._readCommand(self.MCP2515_CMD_STATUS)))
        print('RxStatus = {0:2x}'.format(self._readCommand(self.MCP2515_CMD_RXSTATUS)))
        print('Int      = {0:2x}\n' .format(GPIO.input(self.MCP2515_INT)))

    def DoSendMsg(self, theIdentifier, theData, theLength, theFrameType):
        
        # Write the 11-bit Identifier (the Extended Mode is not implemented
        self._writeReg(self.MCP2515_TXB0SIDH, (theIdentifier >> 3))
        self._writeReg(self.MCP2515_TXB0SIDL, (theIdentifier << 5) & 0xe0)    # The 5 LSB = 0

        # Write the Data Length (4 LSB) and the RTR (Remote Transmission Request) bit
        if (theFrameType == self.MCP2515_TX_STD_FRAME):
            self._writeReg(self.MCP2515_TXB0DLC, (theLength & 0x0f))    # RTR = 0
        elif (theFrameType == self.MCP2515_TX_REMOTE_FRAME):
            self._writeReg(self.MCP2515_TXB0DLC, (theLength | 0xf0))    # RTR = 1
        else:
            prinf("Error : Illegal FrameType in DoSendMsg")
            return (False)

        # Write the Data
        i = 0
        x = self.MCP2515_TXB0D0
        while (i < theLength):
            self._writeReg(x, theData[i])
            x += 1
            i += 1

        # Send Message (TXREQ = 1) with highest priority (TXP = 11)
        self._writeReg(self.MCP2515_TXB0CTRL, 0x0f)
        self.transmit_ready = False
        
        return(True)

    def DoReceiveMsg(self):
        TheRxStatus = self._readCommand(self.MCP2515_CMD_RXSTATUS)

        if ((TheRxStatus & 0xc0) == 0): return (False)

        if ((TheRxStatus & 0x40) != 0):         # Message in RXB0
            print('Message in RXBO')
            print('Mst Type = {0:2x}'.format((TheRxStatus >> 3) & 0x07))
            print('RXB0CTRL = {0:2x}'.format(self._readReg(self.MCP2515_RXB0CTRL)))
            print('RXB0SIDH = {0:2x}'.format(self._readReg(self.MCP2515_RXB0SIDH)))
            print('RXB0SIDL = {0:2x}'.format(self._readReg(self.MCP2515_RXB0SIDL)))
            print('RXB0DLC  = {0:2x}'.format(self._readReg(self.MCP2515_RXB0DLC)))
            print('RXB0D0   = {0:2x}'.format(self._readReg(self.MCP2515_RXB0D0)))
            print('RXB0D1   = {0:2x}'.format(self._readReg(self.MCP2515_RXB0D1)))
            print('RXB0D2   = {0:2x}'.format(self._readReg(self.MCP2515_RXB0D2)))
            print('RXB0D3   = {0:2x}'.format(self._readReg(self.MCP2515_RXB0D3)))
            print('Identif  = {0:2x}'.format((self._readReg(self.MCP2515_RXB0SIDH) << 3) | (self._readReg(self.MCP2515_RXB0SIDL) >> 5)))

        if ((TheRxStatus & 0x80) != 0):       # Message in RXB1
            print('Message in RXB1')
            
        return(True)

    def DoHandleIRQ(self, thePin):
        MyCANINTF = self._readReg(self.MCP2515_CANINTF)

        if ((MyCANINTF & 0x03) != 0):
            print('IRQ from CAN - RX : {0:2x}'.format(MyCANINTF))
            self.DoReceiveMsg()             # Receive the message, just display it !
            self._writeReg(self.MCP2515_CANINTF, self._readReg(self.MCP2515_CANINTF) & 0xfc)     # Clear RX1IF & RX0IF
            
        if ((MyCANINTF & 0x04) != 0):       # TX0E Transmit Buffer 0 Empty
            print('IRQ from CAN - TX : {0:2x}'.format(MyCANINTF))
            self._writeReg(self.MCP2515_CANINTF, self._readReg(self.MCP2515_CANINTF) & 0xfb)     # Clear TX0IF
            self.transmit_ready = True

        if ((MyCANINTF & 0x80) != 0):       
            print('IRQ from CAN - Message Error')
            self._writeReg(self.MCP2515_CANINTF, self._readReg(self.MCP2515_CANINTF) & 0x7f)     # Clear MERRF
        
