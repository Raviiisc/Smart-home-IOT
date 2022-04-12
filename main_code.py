from time import sleep
import os,sys # Sys module is used for interfacing with run time environment, Os module for interfacing with operating system
import RPi.GPIO as GPIO
import paho.mqtt.client as paho # importing the client class
import urlparse
import paho.mqtt.publish as publish
import Adafruit_DHT

DHT_SENSOR = Adafruit_DHT.DHT11

DHT_PIN=19

### Voltage sensor
import RPi.GPIO as GPIO
import time

AO_pin = 0 #flame sensor AO connected to ADC chanannel 0
# change these as desired - they're the pins connected from the
# SPI port on the ADC to the Cobbler
A1_pin=1
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8

#port init
def init():
          GPIO.setwarnings(False)
          GPIO.setmode(GPIO.BCM)
          # set up the SPI interface pins
          GPIO.setup(SPIMOSI, GPIO.OUT)
          GPIO.setup(SPIMISO, GPIO.IN)
          GPIO.setup(SPICLK, GPIO.OUT)
          GPIO.setup(SPICS, GPIO.OUT)
          pass

#read SPI data from MCP3008(or MCP3204) chip,8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)  

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspin, True)
        
        adcout >>= 1       # first bit is 'null' so drop it
        return adcout
##



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
LED_PIN1=26  #define LED pin1
LED_PIN2=27
LIGHTSENSOR_PIN2=13  # define light sensor pin
BUZZ_PIN=5
TOUCH_PIN=6
FAN_PIN=4


GPIO.setup(LED_PIN1,GPIO.OUT)   # Set pin function as output
GPIO.setup(LED_PIN2,GPIO.OUT)   # set pin function as output
GPIO.setup(BUZZ_PIN,GPIO.OUT)
GPIO.setup(FAN_PIN,GPIO.OUT)
GPIO.setup(TOUCH_PIN,GPIO.IN)

GPIO.setup(LIGHTSENSOR_PIN2,GPIO.IN)# set pin function as input
#print GPIO.input(LIGHTSENSOR_PIN2)
    

##

  
voltage=0   
        
 ## Paho.client creates a client class
mqttc = paho.Client()                        # object declaration



url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.emqx.io:1883') 
url = urlparse.urlparse(url_str)
mqttc.connect(url.hostname, url.port)
        
def on_connect(self, mosq, obj, rc):# 
    self.subscribe("led2", 0)
    self.subscribe("led1", 0)
    self.subscribe("fan",0)
   # print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
       # self.subscribe("led2",0)
def on_message(mosq, obj, msg):
     print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
     humidity,temperature=Adafruit_DHT.read(DHT_SENSOR,DHT_PIN)
     if humidity is not None and temperature is not None:
        print("temp={0:0.1f}C humidity= {1:0.1f}%".format(temperature,humidity))
        
     else:
        print("sensor failure");
        
        
     init()
  #   time.sleep(2)
     print("will detect voltage")
         #while True:
     
     ad_value = readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
     voltage= ad_value*(3.3/1024)*5
     
     mVperAmp=43.56 #for 30A module
     RawValue=readadc(A1_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
     ACSoffset=1650 #2499.538
                  
     curr=(RawValue/1024.0)*3300
     amps=((curr-ACSoffset)/mVperAmp)
     
     
    
     #print("Message received-> " + msg.ledx + " " + str(msg.payload))  # Print a received msg
   #  print("Message received-> " + msg.ledx + " " + str(msg.payload))  # Print a received msg
             
     elif(msg.payload == "on2" or  GPIO.input(LIGHTSENSOR_PIN2)==0):
         #temv=temv+1
         GPIO.output(LED_PIN2,GPIO.LOW)
         

     
     if(msg.payload == "off1"):# or GPIO.input(TOUCH_PIN)==1):
         GPIO.output(LED_PIN1,GPIO.LOW)     
         MQTT_PATH = "test_channel"
       # voltage=on_message(mosq, obj, msg)
         #publish.single(MQTT_PATH,"LED1 on", hostname=url.hostname)
         ad_value = readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
         voltage2= ad_value*(3.3/1024)*5
         publish.single(MQTT_PATH, voltage2, hostname=url.hostname) 

             
     elif(msg.payload == "on1" or temperature>25):
         GPIO.output(LED_PIN1,GPIO.HIGH) 
         #GPIO.output(BUZZ_PIN,GPIO.LOW)
       #  print( "LED1 off")   
         MQTT_PATH = "test_channel"
       # voltage=on_message(mosq, obj, msg)
         #publish.single(MQTT_PATH,"LED1 off", hostname=url.hostname
         ad_value = readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
         voltage2= ad_value*(3.3/1024)*5
         publish.single(MQTT_PATH, voltage2, hostname=url.hostname)
     
     if(msg.payload == "off3"):
         GPIO.output(FAN_PIN,GPIO.HIGH) 
         #GPIO.output(BUZZ_PIN,GPIO.LOW)
       #  print( "LED1 off")   
          #MQTT_PATH = "test_channel"
          
     elif(msg.payload == "on3" or temperature>23):
         GPIO.output(FAN_PIN,GPIO.LOW)
          #MQTT_PATH = "test_channel"


def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))

    
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))
    
        
    
#Assign event callbacks
mqttc.on_message = on_message                          # called as callback
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

temv=1          
rc = 0
while True:
  while rc == 0:
     if(GPIO.input(TOUCH_PIN)==1):
         GPIO.output(BUZZ_PIN,GPIO.HIGH)  
     elif(GPIO.input(TOUCH_PIN)==0):
         GPIO.output(BUZZ_PIN,GPIO.LOW)
     if(GPIO.input(LIGHTSENSOR_PIN2)==0 and temv==1):
         GPIO.output(LED_PIN2,GPIO.LOW)
         temv=2
     elif(GPIO.input(LIGHTSENSOR_PIN2)==1 and temv==1):
         GPIO.output(LED_PIN2,GPIO.HIGH)
         temv=2
 
     import time   
     rc = mqttc.loop()
     time.sleep(0.5)
   #  print("rc: " + str(rc))          



