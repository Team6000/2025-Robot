package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class FloorCommand extends Command {
    private final Elevator elevator;
    private final Arm scrubber;

    public FloorCommand(Elevator elevator, Arm scrubber) {
        this.elevator = elevator;
        this.scrubber = scrubber;

        //don't require scrubber, because we schedule a command that does when we need it
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        var scrubberCommand = scrubber.evacuateCommand();

        if (!scrubber.getIsSafe()) scrubberCommand.schedule();
        
        while (scrubber.getCurrentCommand() == scrubberCommand) {
            
        }
        
        System.out.println("       hi there");
    }
}
