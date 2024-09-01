import matplotlib.pyplot as plt
import numpy as np

# Define the gait phases
phases = ['FL', 'FR', 'BL',  'BR']  # Left Hind, Left Front, Right Front, Right Hind
stance = np.array([
    [0.25, 0.5],  # LH stance phase
    [0.75, 1],  # LF stance phase
    [0, 0.25],  # RF stance phase
    [0.5, 0.75]   # RH stance phase
])

# Create the plot
fig, ax = plt.subplots(figsize=(5, 2))

for i in range(len(phases)):
    ax.broken_barh([(stance[i, 0], stance[i, 1] - stance[i, 0])], (i - 0.4, 0.8), facecolors='black')

# Set the ticks
ax.set_yticks(range(len(phases)))
ax.set_yticklabels(phases)

# Set x-axis limits
ax.set_xlim(0, 1)
ax.set_xticks([0, 0.25, 0.5, 0.75, 1])
ax.set_xticklabels(['0', '0.25', '0.5', '0.75', '1'])

# Set labels
ax.set_xlabel('T')
ax.set_ylabel('')
plt.title('Walk')
# Remove grid and frame
ax.grid(False)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)

plt.savefig('gait_pattern.png', dpi=300, bbox_inches='tight')
