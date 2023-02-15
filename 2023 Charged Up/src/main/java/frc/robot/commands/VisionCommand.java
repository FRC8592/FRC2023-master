package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Vision;

public class VisionCommand extends Command {    
    private Drivetrain drive;
    private Vision vision;
    private Timer timer;

    private TargetType target;

    private double delay = 0;

    private PIDController translatePID;
    private PIDController rotatePID;

    public enum TargetType {
        RETROREFLECTIVE_TAPE(Constants.RETROTAPE_PIPELINE, false),
        APRIL_TAG(Constants.APRILTAG_PIPELINE, false),
        CONE(Constants.CONE_PIPELINE, true),
        CUBE(Constants.CUBE_PIPELINE, true),
        ;

        private int pipeline = 0;
        private boolean isPiece = false;
        TargetType(int pipeline, boolean isPiece) {
            this.pipeline = pipeline;
            this.isPiece = isPiece;
        }

        public void setTracking() {
            NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(pipeline);
        }

        // get the x and y offset angles fom the target
        public Translation2d getOffsetAngles() {
            double x = NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("tx").getDouble(0.0);
            double y = NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("ty").getDouble(0.0);;
            return new Translation2d(x, y);
        }

        // Return the x distance and y difference in absolute position
        public Translation2d getDistancesFromTarget() {
            return new Translation2d();
        }

        public boolean isGamePiece() {
            return isPiece;
        }
    }

    public VisionCommand(Drivetrain drive, Vision vision, TargetType target) {
        this.drive = drive;
        this.vision = vision;
        this.target = target;
        timer = new Timer();
    }

    public VisionCommand(Drivetrain drive, Vision vision, TargetType target, double delay) {
        this(drive, vision, target);
        this.delay = delay;
        translatePID = new PIDController(-0.2, 0, 0);
        rotatePID = new PIDController(Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD);
    }

    @Override
    public void initialize() {
        target.setTracking();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        vision.updateVision();
        if (timer.get() >= delay) {
            if (target.isGamePiece()) {
                double strafe = vision.turnRobot(0.0, translatePID, 0.5);
                drive.drive(new ChassisSpeeds(0, strafe, 0));
                return translatePID.atSetpoint();
            } else {
                double speed = vision.moveTowardsTarget(-0.5, -0.5);
                double turn = vision.turnRobot(1.0, rotatePID, 8.0);
                drive.drive(new ChassisSpeeds(speed, 0.0, turn));
                return rotatePID.atSetpoint();
            }
            // Returns if tracking mode complete (Centered on scoring element or moved towards game piece)
        }
        return false;
    }

    @Override
    public void shutdown() {
        drive.drive(new ChassisSpeeds());
    }
    


}