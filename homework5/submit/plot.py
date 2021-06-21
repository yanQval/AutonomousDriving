import matplotlib.pyplot as plt

with open("velocity_1.txt") as f:
    velocity = []
    t = []

    for i, line in enumerate(f.readlines()):
        velocity.append(float(line))
        t.append(i * 0.01)
    
    plt.xlabel("time (s)")
    plt.plot(t, velocity, label="velocity (m/s)")
    plt.legend()

    plt.savefig("velocity_1.png")
    plt.show()