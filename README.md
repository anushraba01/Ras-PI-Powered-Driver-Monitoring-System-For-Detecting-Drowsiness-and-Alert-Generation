# Ras-PI-Powered-Driver-Monitoring-System-For-Detecting-Drowsiness-and-Alert-Generation
# Ras-PI-Powered-Driver-Monitoring-System-For-Detecting-Drowsiness-and-Alert-Generation
#Importing OpenCV Library for basic image processing functions
import cv2
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
#face_utils for basic operations of conversion
from imutils import face_utils
import time
import RPi.GPIO as GPIO     # Import Library to access GPIO PIN
GPIO.setmode(GPIO.BOARD)    # Consider complete raspberry-pi board
GPIO.setwarnings(False)     # To avoid same PIN use warning


# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 11
LCD_D4 = 12
LCD_D5 = 13
LCD_D6 = 15
LCD_D7 = 16
buzzer_pin = 36                # Define PIN for LED
LED_PIN = 29                # Define PIN for LED
'''
define pin for lcd
'''
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
delay = 1



GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(buzzer_pin,GPIO.OUT)   # Set pin function as output
GPIO.setup(LED_PIN,GPIO.OUT)   # Set pin function as output
# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

'''
Function Name :lcd_init()
Function Description : this function is used to initialized lcd by sending the different commands
'''
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
'''
Function Name :lcd_byte(bits ,mode)
Fuction Name :the main purpose of this function to convert the byte data into bit and send to lcd port
'''
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
'''
Function Name : lcd_toggle_enable()
Function Description:basically this is used to toggle Enable pin
'''
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
'''
Function Name :lcd_string(message,line)
Function  Description :print the data on lcd 
'''
def lcd_string(message,line):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)


 
# Define delay between readings
delay = 5

#Initializing the camera and taking the instance
cap = cv2.VideoCapture(0)

#Initializing the face detector and landmark detector
hog_face_detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#status marking for current state
sleep = 0
drowsy = 0
active = 0
status=""
color=(0,0,0)

def compute(ptA,ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a,b,c,d,e,f):
    up = compute(b,d) + compute(c,e)
    down = compute(a,f)
    ratio = up/(2.0*down)

    #Checking if it is blinked
    if(ratio>0.25):
        return 2
    elif(ratio>0.21 and ratio<=0.25):
        return 1
    else:
        return 0

lcd_init()
lcd_string("welcome ",LCD_LINE_1)
time.sleep(2)
lcd_string("Driver Sleep",LCD_LINE_1)
lcd_string("Detection System",LCD_LINE_2)
time.sleep(2)
while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = hog_face_detector(gray)
        #detected face in faces array
        for face in faces:
                x1 = face.left()
                y1 = face.top()
                x2 = face.right()
                y2 = face.bottom()
                face_frame = frame.copy()
                cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                landmarks = predictor(gray, face)
                landmarks = face_utils.shape_to_np(landmarks)

                #The numbers are actually the landmarks which will show eye
                left_blink = blinked(landmarks[36],landmarks[37], 
                landmarks[38], landmarks[41], landmarks[40], landmarks[39])
                right_blink = blinked(landmarks[42],landmarks[43], 
                landmarks[44], landmarks[47], landmarks[46], landmarks[45])

                #Now judge what to do for the eye blinks
                if(left_blink==0 or right_blink==0):
                        sleep+=1
                        drowsy=0
                        active=0
                        if(sleep>1):
                                status="SLEEPING !!!"
                                print("SLEEPING !!!")
                                GPIO.output(buzzer_pin,GPIO.HIGH)
                                GPIO.output(LED_PIN,GPIO.HIGH)
                                lcd_byte(0x01,LCD_CMD)
                                lcd_string("Please wake up ",LCD_LINE_1)
                                time.sleep(0.2)
                                color = (0,0,255)

                elif(left_blink==1 or right_blink==1):
                        sleep=0
                        active=0
                        drowsy+=1
                        if(drowsy>1):
                                status="Drowsy !"
                                GPIO.output(buzzer_pin,GPIO.HIGH)
                                GPIO.output(LED_PIN,GPIO.HIGH)
                                lcd_byte(0x01,LCD_CMD)
                                lcd_string("Please wake up ",LCD_LINE_1)
                                time.sleep(0.2)
                                color = (0,0,255)

                else:
                        drowsy=0
                        sleep=0
                        active+=1
                        if(active>1):
                                status="Active :)"
                                print("Active !!!")
                                lcd_byte(0x01,LCD_CMD)
                                lcd_string("All ok",LCD_LINE_1)
                                lcd_string("Drive Safe",LCD_LINE_2)
                                time.sleep(0.2)
                                GPIO.output(buzzer_pin,GPIO.LOW)
                                GPIO.output(LED_PIN,GPIO.LOW)
                                color = (0,0,255)

                cv2.putText(frame, status, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color,3)

                for n in range(0, 68):
                        (x,y) = landmarks[n]
                        cv2.circle(face_frame, (x, y), 1, (255, 255, 255), -1)

        cv2.imshow("Frame", frame)
        #cv2.imshow("Result of detector", face_frame)
        key = cv2.waitKey(1)
        if key == 27:
                break
