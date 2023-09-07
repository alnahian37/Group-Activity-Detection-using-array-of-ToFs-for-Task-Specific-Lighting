import telnetlib
import time
import numpy as np

# Telnet server IP address and port number
HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.61", "192.168.0.57"]
PORT = 23  # Telnet server port number

# Connect to Telnet servers
# tn1Right = telnetlib.Telnet(HOST[0], PORT)
# tn1Left = telnetlib.Telnet(HOST[1], PORT)
tn2Right = telnetlib.Telnet(HOST[2], PORT)
tn2Left = telnetlib.Telnet(HOST[3], PORT)

start_time = time.perf_counter()  # Record the start time

smoother = 100  # To control how smooth the trasnsition is

array1 = [0]
array2 = [0]

# array1.insert(0, smoother)
# array2.insert(0, smoother)

# Send the array to Arduino as a string
# tn1Right.write(str(array1).encode('utf-8'))
# tn1Left.write(str(array1).encode('utf-8'))
tn2Right.write(str(array2).encode('utf-8'))
tn2Left.write(str(array2).encode('utf-8'))

end_time = time.perf_counter()  # Record the end time

# Closing all the connections
# tn1Right.close()
# tn1Left.close()
tn2Right.close()
tn2Left.close()

# Calculate the elapsed time in milliseconds
elapsed_time = (end_time - start_time) * 1000

# Print the elapsed time in milliseconds
print("Elapsed time: {:.2f} milliseconds".format(elapsed_time))
