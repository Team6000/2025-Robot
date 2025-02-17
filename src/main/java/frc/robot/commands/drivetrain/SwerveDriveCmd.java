package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCmd extends Command {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final SwerveDrive swerveDrive;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;
    
    /**
     * Constructs a new SwerveDriveCmd.
     * 
     * <p>SwerveDriveCmd is used to control the swerve drive base with arcade drive.
     * 
     * @param drive The commanded forward/backward lateral motion.
     * @param strafe The commanded left/right lateral motion.
     * @param rot The commanded rotational motion.
     * @param swerveDrive The required SwerveDrive.
     */
    public SwerveDriveCmd(
        DoubleSupplier drive, 
        DoubleSupplier strafe, 
        DoubleSupplier rot,
        SwerveDrive swerveDrive
    ) {

        this.swerveDrive = swerveDrive;

        this.drive = drive;
        this.strafe = strafe;
        this.rot = rot;

        addRequirements(swerveDrive);

    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double drive = this.drive.getAsDouble();
        drive *= Math.abs(drive);

        double strafe = this.strafe.getAsDouble();
        strafe *= Math.abs(strafe);

        double rot = this.rot.getAsDouble();
        rot *= Math.abs(rot);

        swerveDrive.drive(
            -drive,
            -strafe,
            -rot
        );

    }

}