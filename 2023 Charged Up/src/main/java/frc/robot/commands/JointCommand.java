package frc.robot.commands;

public class JointCommand extends Command {
    private Command[] commands;

    public JointCommand(Command ... pCommands) {
        commands = pCommands;
    }

    @Override
    public void initialize(double pTime) {
        for (Command command : commands) {
            command.initialize(pTime);
        }
    }

    @Override
    public boolean execute() {
        boolean finished = true;
        for (Command command : commands) {
            finished = finished && command.execute();
        }

        return finished;
    }
}
