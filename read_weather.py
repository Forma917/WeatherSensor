import serial
import os
import time
import csv

# name of log file and error file to write to. Specify filepath if need be!
log_file = "overnight_weather_data.csv"
err_file = "overnight_weather_data_errs.csv"

# specify which serial port. May need to change this on RX restart.
ser = serial.Serial(port='COM7',baudrate=115200)

# Format that data ought to be in for the csv.
# Only used for adding a header to a new csv.
data_format=["Time",
             "Node",
             "RSSI",
             "Temp (*C)",
             "Pressure (hPa)",
             "Rel. Humidity (%)",
             "Gas Resistance (KOhm)"]

#Make a header for the log file
if not os.path.exists(log_file):
    with open(log_file,"w") as f:
        csv.writer(f,delimiter=",",lineterminator='\n').writerow(data_format)

# empty the buffer before reading in new data
ser.flushInput()

# only read new data while the csv is less than a GB!
while (os.stat(log_file).st_size<2e30):
    try:
        # read in latest serial line
        serial_in = ser.readline().decode("utf-8")

        # remove unwanted whitespace and newline chars, and convert to list
        node_data="".join(serial_in.split()).split(',')
        # add Julian time to node data for logging
        node_data.insert(0,time.time())
        
        # Try to convert to floats. If fails, could be that 
        # the transmitter is doing a first time handshake message, or throwing errors.
        # Either way, good to know.
        # If data is bad, it gets logged in an errors log file. 
        try:
            node_data=[float(x) for x in node_data]

            # actually go ahead and log the data
            with open(log_file,"a") as f:
                csv.writer(f,delimiter=",",lineterminator='\n').writerow(node_data)
        except:
            with open(err_file,"a") as f:
                csv.writer(f,delimiter=",",lineterminator='\n').writerow(node_data)

        
        

    # using a manual stop function for this script for now
    except:
        print("Keyboard Interrupt")
        break

# dont forget to close the port when you're done!
ser.close()