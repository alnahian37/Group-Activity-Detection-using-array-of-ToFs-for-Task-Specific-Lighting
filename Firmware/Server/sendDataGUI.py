import tkinter as tk
from tkinter import colorchooser
import telnetlib
import time
import numpy as np

def transformMultisUnityToTroffer(listMultis):
    # Scale the multis to 0-4095 and round to integers
    listIntMultis = [int(round((4095*i), 0)) for i in listMultis]
    array = np.array(listIntMultis)
    # Splitting the array into two parts
    edgeMultis = array[:4]
    edgeMultisTransform = np.array([edgeMultis[2], edgeMultis[0], edgeMultis[1], edgeMultis[3]])
    # edgeMultisTransform = np.array([0,0,0,0])
    print(edgeMultisTransform)
    pixelMultis = array[4:]
    pixelMatrix = pixelMultis.reshape((7, 7))
    pixelMatrix = np.flip(pixelMatrix, axis=0)  # Flip the pixel matrix upside down
    print(pixelMatrix)
    mergedMultis = np.concatenate((edgeMultisTransform, pixelMatrix.flatten()), axis=0)
    mergedMultisList = mergedMultis.tolist()  # Flatten the matrix to a list
    return mergedMultisList

def create_array():
    smoother_value = smoother_entry.get()
    edge_value = edge_entry.get()
    spot_value = spot_entry.get()
    
    # Create the array with the specified elements
    array = [float(edge_value)] * 4 + [float(spot_value)] * 49

    array = transformMultisUnityToTroffer(array)

    # Convert the entire array to a string array
    array = list(map(str, array))
    
    # Get the RGB values from the color pickers
    rgb_edge = color_edge_button.cget("background")
    rgb_spot = color_spot_button.cget("background")
    
    # Convert RGB values to integers and add them to the beginning of the array
    array = [smoother_value] + [int(rgb_spot[1:3], 16), int(rgb_spot[3:5], 16), int(rgb_spot[5:7], 16)] + \
            [int(rgb_edge[1:3], 16), int(rgb_edge[3:5], 16), int(rgb_edge[5:7], 16)] + array

    # print(array)

    converted_array = [int(i) for i in array]

    # Send the array to the Arduino
    sendData(converted_array)

def sendData(data):
    # Send the data to the Arduino
    print("Sending data to the Arduino...")

    # Send the array to Arduino as a string
    # tn1Right.write(str(array1).encode('utf-8'))
    # tn1Left.write(str(array1).encode('utf-8'))
    tn2Right.write(str(data).encode('utf-8'))
    tn2Left.write(str(data).encode('utf-8'))


# Create the main GUI window
root = tk.Tk()
root.title("Array Creator")

# Input fields for the parameters with default values
default_smoother_value = "200"
smoother_label = tk.Label(root, text="Smoother:")
smoother_label.grid(row=0, column=0, padx=5, pady=5)
smoother_entry = tk.Entry(root)
smoother_entry.insert(0, default_smoother_value)
smoother_entry.grid(row=0, column=1, padx=5, pady=5)

default_edge_value = "1"
edge_label = tk.Label(root, text="Edge:")
edge_label.grid(row=1, column=0, padx=5, pady=5)
edge_entry = tk.Entry(root)
edge_entry.insert(0, default_edge_value)
edge_entry.grid(row=1, column=1, padx=5, pady=5)

default_spot_value = "0.02"
spot_label = tk.Label(root, text="Spot:")
spot_label.grid(row=2, column=0, padx=5, pady=5)
spot_entry = tk.Entry(root)
spot_entry.insert(0, default_spot_value)
spot_entry.grid(row=2, column=1, padx=5, pady=5)

# Color pickers for RGB values with default colors

def choose_edge_color():
    color = colorchooser.askcolor()[1]
    color_edge_button.config(background=color)

def choose_spot_color():
    color = colorchooser.askcolor()[1]
    color_spot_button.config(background=color)

default_edge_color = "#FF0000"  # Red
color_edge_label = tk.Label(root, text="RGB Edge:")
color_edge_label.grid(row=3, column=0, padx=5, pady=5)
color_edge_button = tk.Button(root, bg=default_edge_color, command=choose_edge_color)
color_edge_button.grid(row=3, column=1, padx=5, pady=5)

default_spot_color = "#00FF00"  # Green
color_spot_label = tk.Label(root, text="RGB Spot:")
color_spot_label.grid(row=4, column=0, padx=5, pady=5)
color_spot_button = tk.Button(root, bg=default_spot_color, command=choose_spot_color)
color_spot_button.grid(row=4, column=1, padx=5, pady=5)

# Button to create the array and run other lines of code
create_button = tk.Button(root, text="Create Array", command=create_array)
create_button.grid(row=5, column=0, columnspan=2, padx=5, pady=10)

# Telnet server IP address and port number
# HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.61", "192.168.0.57"]
# HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.46", "192.168.0.24"]
# HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.63", "192.168.0.55"]
# HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.54", "192.168.0.23"]
HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.61", "192.168.0.57"]
PORT = 23  # Telnet server port number

# Connect to Telnet servers
# tn1Right = telnetlib.Telnet(HOST[0], PORT)
# tn1Left = telnetlib.Telnet(HOST[1], PORT)
tn2Right = telnetlib.Telnet(HOST[2], PORT)
tn2Left = telnetlib.Telnet(HOST[3], PORT)

# Start the main event loop
root.mainloop()

# Closing all the connections
# tn1Right.close()
# tn1Left.close()
tn2Right.close()
tn2Left.close()