package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
public class Climb extends Command {
    private final ClimberSubsystem climber;

    public Climb(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.climberBottomPosition();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
