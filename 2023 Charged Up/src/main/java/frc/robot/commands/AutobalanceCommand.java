package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autopark;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class AutobalanceCommand extends Command {
    private Drivetrain drive;
    private Autopark autopark;
    private Timer timer;
    private double prevPitch = 0;
    private double cyclesWithNoPitchChange = 0;

    private Pose2d startPose;
    
    private boolean cyclesExceeded = false;
    private boolean distanceExceeded = false;
    
    private final double kMinimumDeltaPitch = 5; // Degrees
    private final double kMaximumDistance = 3; // Meters
    private final double kMaximumTimeWithoutPitchChange = 2; // Seconds
    private final double kMaximumCyclesWithoutPitchChange = kMaximumTimeWithoutPitchChange/0.02;

    public AutobalanceCommand(Drivetrain drive) {
        this.drive = drive;
    }

    public AutobalanceCommand(Drivetrain drive, String tag) {
        this.drive = drive;
        setTag(tag);
    }

    @Override
    public void initialize() {
        autopark = new Autopark();
        timer = new Timer();
        timer.reset();
        timer.start();

        startPose = drive.getCurrentPos();
    }

    @Override
    public boolean execute() {
        if (Robot.isReal()) {
            double pitch = drive.getPitch();
            double deltaPitch = Math.abs(pitch - prevPitch);
            Pose2d currentPose = drive.getCurrentPos();
            Transform2d deltaPose = currentPose.minus(startPose);

            if (deltaPose.getX() >= kMaximumDistance || deltaPose.getY() >= kMaximumDistance) {
                distanceExceeded = true;
            }

            if (deltaPitch >= kMinimumDeltaPitch) {
                cyclesWithNoPitchChange++;
            } else {
                cyclesWithNoPitchChange = 0;
            }

            if (cyclesWithNoPitchChange >= kMaximumCyclesWithoutPitchChange*cyclesWithNoPitchChange) {
                cyclesExceeded = true;
            }

            boolean park = !autopark.balance(drive);

            // SmartDashboard.putBoolean("Distance Exceeded", distanceExceeded);
            // SmartDashboard.putBoolean("Cycles Exceeded", cyclesExceeded);
            // SmartDashboard.putBoolean("Park", park);

            prevPitch = pitch;
            return park || /*cyclesExceeded || */ distanceExceeded;
        } else {
            return timer.get() >= 5d;
        }
    }

    @Override
    public void shutdown() {
        drive.setWheelLock();
    }
}
