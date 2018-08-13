### Robot

[Nightfall, our 2017/18 FTC season robot](https://www.youtube.com/watch?v=vCFBw3pLAIE)

### Performance

At the 2018 Texas UIL state championship, Nightfall successfully
* Scored preloaded glyph in 10/10 games
* Collected additional glyphs in 8/10 games, where 2 of those collections were 2-glyph
* Scored additional glyphs in 6/10 games

Failed multiglyph collections were due to unlucky arrangement of the pit. Failed multiglyph deliveries were due to glyphs not being properly aligned in the cargo bay.

### Software overview

`control.AutonomousSuite` is the primary driver class for the autonomous. Given a designated balancing stone and a hardware map, the suite assumes complete control of the robot throughout the phase.

Throughout the design process, an emphasis was placed on time optimization through concurrency without actual multithreading. Many instruction implementations allow additional, nested instructions to be executed within them, and an overarching concurrency schedule allows code that doesn't fit into the main instruction sequence to be executed at specific moments in time.

Cryptobox alignment was achieved by inserting a long arm equipped with sensors in between the cryptobox rails (referred to as dividers in the code). One ODS measured distance from the polycarb backing, and another measured distance from the rail. The robot will purposely undershoot (or overshoot, depending on its alliance color) the target column to ensure that the arm sensors lock onto the correct rail (referred to as latching in the code).

This sensor arm and alignment algorithm design were awarded the Robot Design Award.

Additional ODSs in the cargo bay allowed for persistent glyph gathering. The robot would continue to push itself into the glyph pit until either time ran out or it collected something. Unfortunately, the short redesign period before UIL did not allow for the development of a far plate multiglyph, and the close plate multiglyph was only tuned for one trip.

Driving was perfected with two PIDs; one which controlled velocity, and another which controlled heading. The synergy between these two controllers made for impressive [self-correcting driving](https://www.youtube.com/watch?v=FVmmPYAYJk8&feature=youtu.be).

### Problems

All but one of the robot's motions are guided by sensors: a 4-inch left strafe adjustment made prior to relatching with the cryptobox for a multiglyph delivery. If the robot were to become significantly displaced parallel to the cryptobox during glyph gathering, the sensor arm could collide with a rail on realignment or miss the cryptobox entirely. Though this was never observed, likely thanks to strong PID control, it was still technically possible.

If the robot ever lost contact with the ground, the sudden change in wheel velocities could trick the PID controllers into making false corrections. If the loss of contact lasted long enough, the velocity controller would go unregulated and send the robot flying forward at maximum speed indefinitely. This was observed once in practice when a misplaced glyph became lodged under the jewel collector and lifted the back half of the robot several inches off the ground. Fortunately, this malfunction was caused by something so unusual that it was safe to assume it would never happen in competition.

### Hindsight

This code was written under massive time constraints and is by no means exemplar. If I had anticipated how large and messy the main class would become, I would have organized subsystems and instructions into their own classes.

Using computer vision to guide the parallel movement prior to cryptobox alignment would have eliminated the aforementioned state unlink possibility. This was our original intention, hence the phone's ability to swivel.

Had we planned for odometry wheels in the drivetrain design, I would have liked to use Kalman filtering and more explicit motion profiling to boost localization accuracy.
