
import matplotlib.pyplot as plt

fig = plt.figure(figsize = (10,10))
ax = plt.axes(projection='3d')
ax.grid()

x = []
y = []
z = []
for line in open('/home/ana/Desktop/test.txt', 'r'):
    lines = [i for i in line.split()]
    x.append(float(lines[0]))
    y.append(float(lines[1]))
    z.append(float(lines[2]))


ax.scatter(x, y, z, c = 'r', s = 50)
ax.set_title('3D Scatter Plot')

# Set axes label
ax.set_xlabel('x', labelpad=20)
ax.set_ylabel('y', labelpad=20)
ax.set_zlabel('z', labelpad=20)

plt.show()

