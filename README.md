# Trashboat-Pathfinding
This repository contains three versions of the pathfinding algorithm I developed for Trashboat, an autonomous vehicle to collect trash from water

## How to use

"maintester3.py" is the most recent version of this file, so I will be basing this section on that. Ex Input: startposx(double), startposy(double), angle(double), velocity(m/s), unitOFMeasure(m) (2.6, 7.4, 326.02, 0.4, 1). These are converted coordinates, the grid can be set to how many ever units in the code and the unitOfMeasure describes that conversion. As the boat travels, the boat may come across obstacles and trash. This algorithm takes this into account by continuously changing angles to be as precise as possible and integrating an obstacle avoidance algorithm. The boat goes in a snake-like pattern until the endpoint is reached. 
