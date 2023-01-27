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

    public CommandQueue(Command ... commands) {
        queue = new LinkedList<Command>();
        for (Command command : commands) {
            queue.add(command);
        }

        commandArray = commands;

        timer = new Timer();
    }

    public void initialize() {
        timer.reset();
        timer.start();
        queue.peek().initialize(timer.get());
    }

    public void run() {
        if (!isFinished()) {
            SmartDashboard.putString("Current Running Command", queue.peek().tag() == "" ? "DEFAULT COMMAND" : queue.peek().tag());

            if (queue.peek().execute()) {
                queue.peek().shutdown();
                queue.poll();
                timer.reset();
                timer.start();
                if (queue.size() != 0) {
                    queue.peek().initialize(timer.get());
                } else {
                    SmartDashboard.putString("Current Running Command", "NO COMMAND");
                }
            }
        }
    }

    public boolean isFinished() {
        return queue.size() <= 0;
    }

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
}
