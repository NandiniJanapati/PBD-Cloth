Nandini Janapati
I've completed all portions of the assignment to my knowledge. 
I was not able to figure out the reason, but my simulation runs slowly, even with cloths with small dimensions.

What is a reasonable range for the compliance parameter? What happens outside of this range?
0-0.1 seems to be reasonable. When alpha is greater than 0.1, the cloth becomes too stretchy and it looks unnatural. It sinks too far into the ground.

What is a reasonable range for the damping parameter? What happens outside of this range?
I don't really seem to notice a difference when the damping is 0. I think a reasonable range would be 0 - 1e-3. When d is greater than 1e-3, the cloth moves too slowly and it doesn't seem to settle perpendicualr to the ground quickly enough.

What is a reasonable range for the time step parameter? What happens outside of this range?
h=1e-3 - 1e-2 seems to be the only values that works for me.  When h=1e-1, the cloth disappears. When h=1e-4, the simulation runs too slowly.


What is the highest resolution you can simulate on your computer?
When the cloth is 30x30, the simulation runs slowly but it still runs at a reasonable pace. When the cloth is 40x40, the simulation runs very slowly.