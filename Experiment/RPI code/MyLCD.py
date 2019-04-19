import smbus
import time

class I2C_LCD(object):

    DEVICE_BUS = 1
    DEVICE_ADDR = 0x3E

    # Commands
    CMD_Clear_Display           = 0x01
    CMD_Return_Home             = 0x02
    CMD_Entry_Mode              = 0x04
    CMD_Display_Control         = 0x08
    CMD_Cursor_Display_Shift    = 0x10
    CMD_Function_Set            = 0x30
    CMD_DDRAM_Set               = 0x80
    CMD_Internal_OSC_Freq       = 0x14
    CMD_Power_ICON_Contrast     = 0x50
    CMD_Follower_Control        = 0x60
    CMD_Contrast_Set            = 0x70

    # Options
    OPT_Increment       = 0x02      # CMD_Entry_Mode
    OPT_Display_Shift   = 0x01      # CMD_Entry_Mode
    OPT_Enable_Display  = 0x04      # CMD_Display_Control
    OPT_Enable_Cursor   = 0x02      # CMD_Display_Control
    OPT_Enable_Blink    = 0x01      # CMD_Display_Control
    OPT_Display_Shift   = 0x08      # CMD_Cursor_Display_Shift
    OPT_Shift_Right     = 0x04      # CMD_Cursor_Display_Shift 0 = Left
    OPT_2_Lines         = 0x08      # CMD_Function_Set 0 = 1 line
    OPT_Instr_Table     = 0x01      # CMD_Function_Set IS Bit
    OPT_5x10_Dots       = 0x04      # CMD_Function_Set 0 = 5x7 dots
    OPT_Booster_On      = 0x04      # Enable Booster Circuit for 3.3 V
    OPT_Follower        = 0x0f      # Enable Follower and Set Amplified Ratio for 3.3 V
    OPT_Contrast_LSB    = 0x04      # LSB Contrast Bits for 3.3V

    def __init__(self):
        self.bus = smbus.SMBus(self.DEVICE_BUS)
        
        self.command(self.CMD_Function_Set | self.OPT_2_Lines)
        self.command(self.CMD_Function_Set | self.OPT_2_Lines | self.OPT_Instr_Table)
        self.command(self.CMD_Internal_OSC_Freq)
        self.command(self.CMD_Contrast_Set | self.OPT_Contrast_LSB)
        self.command(self.CMD_Power_ICON_Contrast | self.OPT_Booster_On)
        self.command(self.CMD_Follower_Control | self.OPT_Follower)
        self.command(self.CMD_Display_Control | self.OPT_Enable_Display)
        self.command(self.CMD_Clear_Display)
        self.command(self.CMD_Entry_Mode | self.OPT_Increment |  self.OPT_Display_Shift)

    def _write(self, data, command=True):
        if command:
            self.bus.write_byte_data(self.DEVICE_ADDR, 0x00, data)
        else:
            self.bus.write_byte_data(self.DEVICE_ADDR, 0x40, data)

    def command(self, data):
        self._write(data)
        time.sleep(0.1)

    def clear(self):
        self.command(self.CMD_Clear_Display)
	
    def home(self):
        self.command(self.CMD_Return_Home)

    def setPosition(self, line, pos):
        if line == 1:
            address = pos
        elif line == 2:
            address = 0x40 + pos
        self.command(self.CMD_DDRAM_Set + address)
	
    def writeChar(self, char):
        self._write(ord(char), False)

    def writeString(self, string):
        for c in string:
            self.writeChar(c)
