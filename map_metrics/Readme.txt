This project allows you to calculate some map metrics:
1. Number of corners in the estimated map (Harris)
2. Number of enclosed areas in the estimated map (Suzuki)
3. Distance betwen estimated and original maps, calculated by finding of nearest occuped
sell in the original map for every occuped sell in the estimated map, and then 
summing of distanses between them (KNN)


Both maps must be aligned before usage.

Usage:
harris_suzuki estimated_map.png original_map.png
