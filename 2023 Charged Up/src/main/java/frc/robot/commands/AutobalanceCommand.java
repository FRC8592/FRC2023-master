package frc.robot.commands;

import frc.robot.Drivetrain;

public class AutobalanceCommand extends Command {
    private Drivetrain drive;

    public AutobalanceCommand(Drivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean execute() {
        return false;
    }

    @Override
    public void shutdown() {
        
    }
    
}
