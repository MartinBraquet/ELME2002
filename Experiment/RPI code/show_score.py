import sys
import MyLCD

MyLCD = MyLCD.I2C_LCD()
MyLCD.writeString(str("Score: " + sys.argv[1]))
