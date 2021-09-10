import csv
import numpy as np
import matplotlib.pyplot as plt

simulation_count = np.array([]); fitness_max =np.array([]); fitness_min = np.array([]); fitness_ave = np.array([])

with open('Finess-2.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        simulation_count = np.append(simulation_count, row[0])
        fitness_max = np.append(fitness_max, row[1])
        fitness_min = np.append(fitness_min, row[2])
        fitness_ave = np.append(fitness_ave, row[3])
        line_count += 1

simulation_count = simulation_count.astype(float)
fitness_max = fitness_max.astype(float)
fitness_min = fitness_min.astype(float)
fitness_ave = fitness_ave.astype(float)

plt.plot(simulation_count, fitness_max,'-x', label='fitness_max')
plt.plot(simulation_count, fitness_min,'-x', label='fitness_min')
plt.plot(simulation_count, fitness_ave,'-x', label='fitness_ave')

plt.legend(loc='upper left')
plt.ylabel("Fitness Value")
plt.xlabel("Generation")

plt.show()