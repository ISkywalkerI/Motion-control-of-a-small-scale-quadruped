11378744 Project 

Run the walk1.py to watch the walk gait pattern simulation of the robot.

change 
x,y,z = gait.walk_rotate(xf, xs, yf, ys, zs, height, t, Ts, 0) 
to
x,y,z = gait.trot(xf, xs, yf, ys, zs, height, t, Ts,0) 
can simulate the trot gait pattern.
