# 2025-Robot
REEFSCAPE robot code

[Controller layouts](https://www.padcrafter.com/?templates=Driver%7COperator&plat=0%7C0&col=%2523D3D3D3%2C%25233E4B50%2C%2523FFFFFF&yButton=Speed+Up%7C&aButton=Slow+Down%7C&dpadUp=%7C&leftTrigger=Lock%7C&leftStick=X%2FY+Movement%7C&rightStickClick=Rotation%7C&startButton=Toggle+Field+Oriented%7C&backButton=Reset+Heading%7C&dpadRight=Align+to+right+pole+of+nearest+side+of+Reef%7C&dpadLeft=Align+to+left+pole+of+nearest+side+of+Reef%7C&rightTrigger=%7C)

![alt text](image.png)

![alt text](image-1.png)

## TODO
- [ ] Swerve [NEW: MK4n!]
  - [ ] Verify accurate IDs
  - [ ] Measure wheel locations (should be square)
  - [ ] Ensure PID works
  - [ ] Current limits, friction, etc
- [ ] Elevator
  - [ ] Write infrastructure
    - [ ] Feedforward, PID
    - [ ] Setpoints
    - [ ] Manual control: total? slight adjustments?
    - [ ] Interactions with Algae Manipulator
  - [ ] SysID
    - [ ] Fast, but not too fast
- [ ] Shooter
  - [ ] Write infrastructure
  - [ ] Determine good speed
- [ ] Algae Manipulator (find better name?)
  - [ ] Write infrastructure
    - [ ] Part of Shooter.java? Own class?
  - [ ] Location/setpoint control -- Feedforward. PID probably not necessary
  - [ ] CRITICAL: software limits: prevent collision with shooter, bumper, elevator
  - [ ] Determine good speed for outer wheels
- [ ] Vision/Driving
  - [ ] Include AprilTags via LL2 in optometry
  - [ ] Driver assist: [nudge/take over] for lining up with Reef
  - [ ] IT DEPENDS: flight stick controls
  - [ ] OPTIONAL: driver buttons for juking, other stuff
- [ ] Autonomous
  - [ ] Create paths, match up commands

### Resources

[Introduction to System Identification](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)

[Controls](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html)

[Advanced Controls Introduction](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html)

[PID Control in Command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html)

[Limelight docs](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)

#### Swerve code copied from [SwerveBot](https://github.com/FIREBOTICS/SwerveBot).