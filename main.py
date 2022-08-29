import os, time
import serial               
from time import sleep
import webbrowser           
import sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
x=0 
map_link =""

import time
L1 = 5
L2 = 18
L3 = 22
L4 = 19

# These are the four columns
C1 = 29
C2 = 31
C3 = 23
C4 = 21
 
# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 11
LCD_D4 = 12
LCD_D5 = 13
LCD_D6 = 15
LCD_D7 = 16
LED_ON = 8

alcohol_Sensor = 18
led=38
relay = 32
vibration_sensor = 33
# Define some device constants
LCD_WIDTH = 20    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
 
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line
 
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005





GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(LED_ON, GPIO.OUT) # Backlight enable
GPIO.setup(led, GPIO.OUT)
GPIO.setup(relay, GPIO.OUT)
GPIO.setup(alcohol_Sensor, GPIO.IN)
GPIO.setup(vibration_sensor, GPIO.IN) 


# Setup GPIO

GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(L3, GPIO.OUT)
GPIO.setup(L4, GPIO.OUT)

# Use the internal pull-down resistors
GPIO.setup(C1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialise display

 

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
 
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
 
  GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
 
def lcd_string(message,line,style):
  # Send string to display
  # style=1 Left justified
  # style=2 Centred
  # style=3 Right justified
 
  if style==1:
    message = message.ljust(LCD_WIDTH," ")
  elif style==2:
    message = message.center(LCD_WIDTH," ")
  elif style==3:
    message = message.rjust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
 
def lcd_backlight(flag):

  GPIO.output(LED_ON, flag)
 
lcd_init()
 # Send some centred test
lcd_string("________________",LCD_LINE_1,1)
lcd_string("wellcome",LCD_LINE_2,1)
lcd_string("smart car system",LCD_LINE_3,1)
lcd_string("_________________",LCD_LINE_4,1)
time.sleep(3) 
lcd_byte(0x01, LCD_CMD)
time.sleep(3) 



def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    print("NMEA Time: ", nmea_time,'\n')
    print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
   
gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

# The GPIO pin of the column of the key that is currently
# being held down or -1 if no key is pressed
keypadPressed = -1
input = "0"


# This callback registers the key that was pressed
# if no other key is currently pressed
def keypadCallback(channel):
    global keypadPressed
    if keypadPressed == -1:
        keypadPressed = channel

# Detect the rising edges on the column lines of the
# keypad. This way, we can detect if the user presses
# a button when we send a pulse.
GPIO.add_event_detect(C1, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C2, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C3, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C4, GPIO.RISING, callback=keypadCallback)

# Sets all lines to a specific state. This is a helper
# for detecting when the user releases a button
def setAllLines(state):
    GPIO.output(L1, state)
    GPIO.output(L2, state)
    GPIO.output(L3, state)
    GPIO.output(L4, state)
def checkSpecialKeys():
    global input
    global x
    
    pressed = False

    GPIO.output(L4, GPIO.HIGH)

    if (GPIO.input(C1) == 1):
        lcd_string("input reset",LCD_LINE_1,1)
        
        pressed = True

    if (not pressed and GPIO.input(C3) == 1):
        if len(input)==10:
            lcd_string("phone:"+input,LCD_LINE_1,1)
            time.sleep(3)
            x=1

        else:
            lcd_string("enter again",LCD_LINE_1,1)

            pressed = True

    GPIO.output(L3, GPIO.LOW)

    if pressed:
        input = "0"

    return x
# to the button, to a variable
def readLine(line, characters):
    global input
    # We have to send a pulse on each line to
    # detect button presses
    GPIO.output(line, GPIO.HIGH)
    if(GPIO.input(C1) == 1):
        input = input + characters[0]
        lcd_string(input,LCD_LINE_1,1)
        
    if(GPIO.input(C2) == 1):
        input = input + characters[1]
        lcd_string(input,LCD_LINE_1,1)
        
    if(GPIO.input(C3) == 1):
        input = input + characters[2]
        lcd_string(input,LCD_LINE_1,1)
        
    if(GPIO.input(C4) == 1):
        input = input + characters[3]
        lcd_string(input,LCD_LINE_1,1)
        
    GPIO.output(line, GPIO.LOW)
try:
    while True:
          vibration_data = GPIO.input(vibration_sensor)
          alcohol_data = GPIO.input(alcohol_Sensor)
          received_data = (str)(ser.readline())  # read NMEA string received
          GPGGA_data_available = received_data.find(gpgga_info)  # check for NMEA GPGGA string
          if (GPGGA_data_available > 0):
                GPGGA_buffer = received_data.split("$GPGGA,", 1)[1]  # store data coming after "$GPGGA," string
                NMEA_buff = (GPGGA_buffer.split(','))  # store comma separated data in buffer
                GPS_Info()  # get time, latitude, longitude
                map_link = 'http://maps.google.com/?q=' + lat_in_degrees + ',' + long_in_degrees  # create link to plot location on Google map

          if(x==0):
                 if keypadPressed != -1:
                      setAllLines(GPIO.HIGH)
                      if GPIO.input(keypadPressed) == 0:
                         keypadPressed = -1
                      else:
                         time.sleep(0.1)
                # Otherwise, just read the input
                 else:
                     while (not checkSpecialKeys()):
                        readLine(L1, ["1","2","3","A"])
                        readLine(L2, ["4","5","6","B"])
                        readLine(L3, ["7","8","9","C"])
                        readLine(L4, ["*","0","#","D"])
                        time.sleep(0.1)
                     time.sleep(0.1)
          if (x==2):

                 if (vibration_data == False):
                          if (alcohol_data == True):
                              lcd_string("car stoped ",LCD_LINE_1,1)
                              GPIO.output(relay, False)
                              GPIO.output(led, True)
                              time.sleep(0.5)
                              GPIO.output(led, False)
                              time.sleep(0.5)
                          else:
                                lcd_byte(0x01,LCD_CMD) # 000001 Clear display
                                lcd_string("Vehicle Start ",LCD_LINE_1,1)
                                time.sleep(0.5)
                                GPIO.output(led, False)
                                GPIO.output(relay, True)
                 else:
                           port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
                           port.write(b'AT\r')
                           time.sleep(1)
                           port.write(b"AT+CMGF=1\r")
                           time.sleep(3)
                           phone= ("AT+CMGS=\" "+str(input)+"\"\n")
                           port.write(phone.encode())
                           msg = map_link
                           time.sleep(3)
                           port.reset_output_buffer()
                           time.sleep(1)
                           port.write(str.encode(msg+chr(26)))
                           time.sleep(3)
                           break

                # If a button was previously pressed,
                    # check, whether the user has released it yet

except KeyboardInterrupt:
    print("\nApplication stopped!")

