package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;

public class CommandQueue {
    private Queue<Command> queue;
    private Timer timer;
    private double prevTime = 0;

    public CommandQueue(Command ... commands) {
        queue = new LinkedList<Command>();
        for (Command command : commands) {
            queue.add(command);
        }

        timer = new Timer();
    }

    public void initialize() {
        timer.reset();
        timer.start();
        // for (Command command : queue) {
        //     command.initialize(timer.get());
        // }
        queue.peek().initialize(0);
    }

    public void run() {
        if (!isFinished()) {
            if (queue.peek().execute()) {
                queue.poll();
                timer.reset();
                timer.start();
                if (queue.size() != 0) {
                    queue.peek().initialize(timer.get());
                }
            }
        }
    }

    public boolean isFinished() {
        return queue.size() <= 0;
    }
}
