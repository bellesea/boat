import geojson
import serial
import time
## PROCESS DATA
with open('./test.geojson') as f:
    gj = geojson.load(f)

coordinates = gj['features'][0]['geometry']['coordinates']

processed_coordinates = []

for group in coordinates:
    for c in group:
        coordinate = [c[0], c[1]]
        processed_coordinates.append(coordinate)

## SEND DATA
arduinoComPort = "/dev/cu.usbmodem2101"
baudRate = 9600

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1) 

def write_read(x): 
    arduino.write(bytes(x, 'utf-8')) 
    time.sleep(0.05) 
    data = arduino.readline() 
    return data 

for c in processed_coordinates:
    lat = write_read(c[0])
    lng = write_read(c[1])
    print(lat, lng)