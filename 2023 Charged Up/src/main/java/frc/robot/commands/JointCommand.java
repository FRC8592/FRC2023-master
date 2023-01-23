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
            if (!command.execute()) {
                finished = false;
            }
        }

        return finished;
    }

    public Command[] getCommands() {
        return commands;
    }

    @Override
    public void shutdown() {
        for (Command command : commands) {
            command.shutdown();
        }        
    }
}
