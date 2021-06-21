import matplotlib.pyplot as plt
from math import *

plt.figure(figsize=(10, 10))

file = open("output_data_5.txt", "r")

n = int(file.readline())
x = []
y = []
for i in range(n - 300):
    x.append(float(file.readline()))
    y.append(float(file.readline()))

plt.scatter(x, y, c = 'red', marker = 'o', label = 'input')

intp_x = []
intp_y = []
for i in range(300):
    intp_x.append(float(file.readline()))
    intp_y.append(float(file.readline()))

#print(intp_x)
#print(intp_y)

plt.scatter(intp_x, intp_y, s = 20, c = 'lightcoral', marker = '.', label = 'interpolated')

plt.legend()

plt.show()

file.close