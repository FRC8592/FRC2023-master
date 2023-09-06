package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommandQueue {
    private Queue<Command> queue;

    private Command[] commandArray;
    private Timer timer;

    private int counter = 1;

    /**
     * @param commands list of commands (in order) to run for the queue
     */
    public CommandQueue(Command ... commands) {
        queue = new LinkedList<Command>();
        for (Command command : commands) {
            queue.add(command);
        }

        commandArray = commands;
        timer = new Timer();
        timer.reset();
        timer.start();
        queue.peek().initialize();
    }

    /**
     * Starts the queue
     */
    public void initialize() {
        timer.reset();
        timer.start();
        queue.peek().initialize();
    }

    /**
     * periodic loop ran as long as the queue is not empty
     */
    public void run() {
        if (!isFinished()) {
            // SmartDashboard.putString("Current Running Command", queue.peek().tag() == "" ? "DEFAULT COMMAND" : queue.peek().tag());

            //if command has been executed reset and get ready for the next command
            if (queue.peek().execute()) {
                queue.peek().shutdown();
                queue.poll();
                timer.reset();
                timer.start();
                // SmartDashboard.putNumber("Command Counter", counter);
                counter++;
                if (queue.size() != 0) {
                    queue.peek().initialize();
                } else {
                    // SmartDashboard.putString("Current Running Command", "NO COMMAND");
                }
            }
        }
    }

    /**
     * @return if the queue has completed
     */
    public boolean isFinished() {
        return queue.size() <= 0;
    }

    /**
     * @return {@code Pose2d} element representing the starting position based on queued commands
     */
    public Pose2d getStartPose() {
        for (Command command : commandArray) {
            if (command.getClass() == FollowerCommand.class) {
                return command.getStartPose();
            } else if (command.getClass() == JointCommand.class) {
                for (Command jointCommand : ((JointCommand)command).getCommands()) {
                    if (jointCommand.getClass() == FollowerCommand.class) {
                        return jointCommand.getStartPose();
                    }
                }
            }
        }
        return new Pose2d();
    }

    public Queue<Command> getQueue() {
        return queue;
    }

    public CommandQueue addCommand(Command command) {
        queue.add(command);
        return this;
    }

    public CommandQueue addDelay(double seconds) {
        DelayCommand delay = new DelayCommand(seconds);
        queue.add(delay);
        Command[] newArray = new Command[commandArray.length + 1];
        newArray[0] = delay;
        for (int i = 0; i < commandArray.length; i++) {
            newArray[i+1] = commandArray[i];
        }
        queue = new CommandQueue(newArray).getQueue();
        return this;
    }
}
