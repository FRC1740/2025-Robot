# quick script to visualize the elevator paths and poses
# takes data in the form of cli path points then constraints 
# python ploy.py "p1x,p1y,p2x,p2y,p3x,p3y" "p1x,p1y,p2x,p2y,p1x,p1y,p2x,p2y"

import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches



# total arguments
if len(sys.argv) != 3:
    raise Exception("Incorrect number of args passed")

bezierPoints = sys.argv[1]
constraintPoints = sys.argv[2]
print(bezierPoints)
print(constraintPoints)

constraintPoints = constraintPoints.split(",")

# Convert to integers
constraintPoints = list(map(float, constraintPoints))

bezierPoints = bezierPoints.split(",")

# Convert to integers
bezierPoints = list(map(float, bezierPoints))

# Separate x and y coordinates
x = bezierPoints[0::2]  # Elements at even indices (0, 2, 4, ...)
y = bezierPoints[1::2]  # Elements at odd indices (1, 3, 5, ...)


# Add labels and title
plt.xlabel('Elevator Height 0 - 1')
plt.ylabel('Arm Angle in radians')
plt.title('Plot')

# Plot the points
plt.scatter(x, y)  # 'o' indicates a circle marker


for i in range(0, len(constraintPoints), 4):
    group = constraintPoints[i:i + 4] # group all 4 points

    # Define the rectangle parameters: (x, y), width, height
    rect = patches.Rectangle((group[0], group[1]), group[2] - group[0], group[3] - group[1], linewidth=2, edgecolor='r', facecolor='none')

    # Add the rectangle to the plot
    plt.gca().add_patch(rect)

# Show the plot
plt.show()