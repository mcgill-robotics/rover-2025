import serial
import os

port = serial.Serial(port="/dev/ttyACM0", baudrate=9600) # Put the name of the serial port for the final code

valuesDict = {"ph1": "", "ph2": "", "ph3": "", "m1": "", "m2": "", "m3": ""}


def separate_data(strVals):
    dict_vals = {"ph1": "", "ph2": "", "ph3": "", "m1": "", "m2": "", "m3": ""}

    # String pattern: ph1=%d, ph2=%d, ph3=%d, m1=%d, m2=%d, m3=%d\r\n

    list_values = strVals.split(", ")
    # print(list_values)
    for val in list_values:
        for key in dict_vals.keys():
            # print(val)
            pair = val.split("=")
            print(pair[0], key)
            if pair[0] == key:
                print("match")
                dict_vals[key] = int(pair[1])
                print(int(pair[1]))

    return dict_vals

def write_to_csv(filename, value, timestamp):
    # open the csv file from the filename
    full_filename = "data/" + filename + ".csv"
    with open(full_filename,'a') as fd:
        fd.write(str(value) + ", " + timestamp)
        fd.close()

if not os.path.isdir("data"):
    os.mkdir("data")

byte_empty = b'\x00\n'

while True:
    # Read the lines from Serial
    value = port.readline()
    stringValue = str(value, "UTF-8")
    print(stringValue)
    if (value == byte_empty):
        continue
    # Extract RTC value from the payload
    stringValue = stringValue.split("; ")
    timestamp = stringValue[1]
    stringValue = stringValue[0]
    # Convert values into dictionary
    valuesDict = separate_data(stringValue)
    print(valuesDict)
    # Store values into files
    for key, value in valuesDict.items():
        write_to_csv(key, value, timestamp)