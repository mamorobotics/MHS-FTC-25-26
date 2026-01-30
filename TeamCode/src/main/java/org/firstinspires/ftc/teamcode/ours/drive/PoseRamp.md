# Data

In the RoadRunner coordinate system, lauching at -6,6 (approximate - refine this position further), recquires a flywheel speed of 1000 and a neutrlal ramp angle. 

**Table of values:**
x , y , speed, ramp
-6, 6, 950, 0.38
-36, -24, 1100, 0.41 (this value i lowkenuinely guestimated)


**Center of Goal:** (-63, -51) (im guestimating as well)


# Derivation

DISTANCE from the center: sqrt((x+63) ^ 2 + (y+51) ^2)

Implemented in the code

Ran linear regression using data points, between distance and speed / ramp angle

Put m and b in functions in Subsystems.java (y=mx+b)