package frc.robot.commands;

public abstract class Command {
    public abstract void initialize(double pTime);
    public abstract boolean execute();
}