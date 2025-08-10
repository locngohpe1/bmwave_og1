import numpy as np
import matplotlib.pyplot as plt
import csv

# data for algorithm 1 - proposed algorithm
total_path_length1 = []
total_path_length2 = []
number_of_return1 = []
number_of_return2 = []

with open('result.csv', 'r') as file:
    # create a CSV reader
    reader = csv.DictReader(file)

    # print the data
    for row in reader:
        if row['name'][0:9] == 'scenario5':
            total_path_length1.append(round(float(row['total length']), 2))
            number_of_return1.append(int(row['number of return']) - 1)

with open('result_e_star.csv', 'r') as file:
    # create a CSV reader
    reader = csv.DictReader(file)

    # print the data
    for row in reader:
        if row['name'][0:9] == 'scenario5':
            total_path_length2.append(round(float(row['total length']), 2))
            number_of_return2.append(int(row['number of return']) - 1)

# create an array of indices for the bars
# x = ['indoor', 'cave', 'storage']
x = ['Test case 5.1', 'Test case 5.2', 'Test case 5.3']

idx = np.arange(len(total_path_length1))

fig, ax1 = plt.subplots()

# set the width of each bar
bar_width = 0.25

# create the bar chart for path length data
ax1.bar(idx - bar_width/2, total_path_length1, width=bar_width, label="BWave", color="#F08080")
ax1.bar(idx + bar_width/2, total_path_length2, width=bar_width, label="ε*+", color="#87CEEB")
ax1.set_xlabel('Real-life simulated environment', fontweight='bold')
ax1.set_ylabel('Total path length (unit)', fontweight='bold')
ax1.legend(loc='best')
# ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=5, frameon=True)

ax1.set_xticks(idx)
ax1.set_xticklabels(x)

# ax2 = ax1.twinx()

# ax2.plot(idx, number_of_return1, marker='o', markersize=5, label="(number of return) Proposed Algorithm", color="#696969")
# ax2.plot(idx, number_of_return2, marker='o', markersize=5, label="(number of return) ε*+", color="#FFFF00")

# ax2.set_yticks([4, 5, 6, 7, 8, 9])
# ax2.set_ylabel('Number of return', fontweight='bold')

# ask matplotlib for the plotted objects and their labels
lines, labels = ax1.get_legend_handles_labels()
# lines2, labels2 = ax2.get_legend_handles_labels()

# ax2.set_xticks(idx)
# ax2.set_xticklabels(x)
# ax2.legend(title='Time', loc='upper right')
# ax2.legend(lines + lines2, labels + labels2, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=5, frameon=False)
# ax2.legend(lines + lines2, labels + labels2, loc='best')

# fig.set_size_inches(h=4, w=8)
plt.title("Scenario 5 - Real-life simulated environment", fontweight='bold')

# show the plot
plt.savefig('draw/img/scenario5.1.png')
plt.show()
