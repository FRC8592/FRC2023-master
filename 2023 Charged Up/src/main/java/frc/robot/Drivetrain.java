package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;


public class Drivetrain {
    /**
     * Swerve module controllers, intialized in the constructor
     */  
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private SwerveDriveOdometry odometry; //Odometry object for swerve drive
    
    private FRCLogger logger;

    private final double kWheelCircumference = 4*Math.PI;
    private final double kFalconTicksToMeters = 1.0 / 4096.0 / kWheelCircumference;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     * TODO: 12.0 volts may be a little high.  Monitor battery voltage to determine if a lower voltage will work better
     */
    private static final double MAX_VOLTAGE = 12.0;

    // Measure the drivetrain's maximum velocity (m/s) or calculate the theoretical maximum.
    //
    // This formula is taken from the SDS swerve-template repository: https://github.com/SwerveDriveSpecialties/swerve-template
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6;
    // 6380.0 / 60.0 *
    //     SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
    //     SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    // The maximum angular velocity of the robot in radians per second.
    //
    // This calculated value could be replaced with a measured value.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    // Set up the kinematics module based on physical drivetrain characteristics
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    // FIXME Remove if you are using a Pigeon
    // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    
    /**Initialize drivetrain
     * 
     */
    public Drivetrain(FRCLogger logger) {
        Mk4ModuleConfiguration swerveMotorConfig;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Create configuration object for motors.  We do this primarily for current limiting
        swerveMotorConfig = new Mk4ModuleConfiguration();
        swerveMotorConfig.setNominalVoltage(MAX_VOLTAGE);
        swerveMotorConfig.setDriveCurrentLimit(ConfigRun.MAX_SWERVE_DRIVE_CURRENT);
        swerveMotorConfig.setSteerCurrentLimit(ConfigRun.MAX_SWERVE_STEER_CURRENT);
        
        this.logger = logger;

        // Create motor objects
        //
        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // Motor configuration
            swerveMotorConfig,
            // This can either be L!, L2, L3 or L4
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
            swerveMotorConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
            swerveMotorConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR, 
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
            swerveMotorConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        );

        this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), new SwerveModulePosition[]  {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()});
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        // FIXME Remove if you are using a Pigeon
        // m_pigeon.setFusedHeading(0.0);
        m_navx.zeroYaw();   // We're using a NavX
    }

    public Rotation2d getGyroscopeRotation() {
        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public boolean isGyroscopeRotating(){
        return m_navx.isRotating();
    }

    public double getYaw(){
        return m_navx.getYaw();
    }

    public Pose2d getCurrentPos(){
        Pose2d pos = odometry.getPoseMeters();
        SmartDashboard.putNumber("Drive X (in)", pos.getX() * 39.3701); //meters to inches
        SmartDashboard.putNumber("Drive Y (in)", pos.getY()  * 39.3701 );
        SmartDashboard.putNumber("Drive Yaw (deg)", pos.getRotation().getDegrees());
        return pos;
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(new Rotation2d(0), new SwerveModulePosition[]  {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, pose);
    }
    
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putNumber("Chassis Speeds X", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Y", chassisSpeeds.vyMetersPerSecond);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        setDriveVelocity(metersPerSecondToTicks(states[0].speedMetersPerSecond), m_frontLeftModule);
        setDriveVelocity(metersPerSecondToTicks(states[1].speedMetersPerSecond), m_frontRightModule);
        setDriveVelocity(metersPerSecondToTicks(states[2].speedMetersPerSecond), m_backLeftModule);
        setDriveVelocity(metersPerSecondToTicks(states[3].speedMetersPerSecond), m_backRightModule);

        m_frontLeftModule.setSteerAngle(states[0].angle.getRadians());
        m_frontRightModule.setSteerAngle(states[1].angle.getRadians());
        m_backLeftModule.setSteerAngle(states[2].angle.getRadians());
        m_backRightModule.setSteerAngle(states[3].angle.getRadians());
        
        this.odometry.update(
            getGyroscopeRotation(), 
            new SwerveModulePosition[] {
                getSMPosition(m_frontLeftModule), 
                getSMPosition(m_frontRightModule),
                getSMPosition(m_backLeftModule),
                getSMPosition(m_backRightModule)
            }
        );

        if (Robot.isReal()) {
            Robot.FIELD.setRobotPose(getCurrentPos());
        }
        //Steer Angles
        SmartDashboard.putNumber("Front Left Azimuth (Degrees)", getSMPosition(m_frontLeftModule).angle.getDegrees());
        SmartDashboard.putNumber("Front Right Azimuth (Degrees)", getSMPosition(m_frontRightModule).angle.getDegrees());
        SmartDashboard.putNumber("Back Left Azimuth (Degrees)", getSMPosition(m_backLeftModule).angle.getDegrees());
        SmartDashboard.putNumber("Back Right Azimuth (Degrees)", getSMPosition(m_backRightModule).angle.getDegrees());

        //Motor Velocities
        SmartDashboard.putNumber("Front Left Velocity", getModuleVelocity(m_frontLeftModule));
        SmartDashboard.putNumber("Front Right Velocity", getModuleVelocity(m_frontRightModule));
        SmartDashboard.putNumber("Back Left Velocity", getModuleVelocity(m_backLeftModule));
        SmartDashboard.putNumber("Back Right Velocity", getModuleVelocity(m_backRightModule));

        //What Velocities we're trying to set
        SmartDashboard.putNumber("Front Left", metersPerSecondToTicks(states[0].speedMetersPerSecond));
        SmartDashboard.putNumber("Front Right", metersPerSecondToTicks(states[1].speedMetersPerSecond));
        SmartDashboard.putNumber("Back Left", metersPerSecondToTicks(states[2].speedMetersPerSecond));
        SmartDashboard.putNumber("Back Right", metersPerSecondToTicks(states[3].speedMetersPerSecond));


        logger.log(this, "SwerveModuleStates", new SwerveModule[] {m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule});
        // logger.log(this, "CANCoder Values", new double[] {m_frontLeftModule.getSteerAngle(), m_frontRightModule.getSteerAngle(), })
    } 

    public void resetEncoder(){
        m_frontLeftModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_frontRightModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_backLeftModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_backRightModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
    }

    public void getSwervePositions() {
        SmartDashboard.putNumber("Front Left Posiiton", m_frontLeftModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        SmartDashboard.putNumber("Front Right Posiiton", m_frontRightModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        SmartDashboard.putNumber("Back Left Posiiton", m_backLeftModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        SmartDashboard.putNumber("Back Right Posiiton", m_backRightModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
    }

    private SwerveModulePosition getSMPosition(SwerveModule mod){
        return new SwerveModulePosition(mod.getDriveController().getDriveFalcon().getSelectedSensorPosition()/4096.0/kWheelCircumference, new Rotation2d(mod.getSteerAngle()));
    }

    public void setDriveVelocity(double inputVelocity, SwerveModule module){
        module.getDriveController().getDriveFalcon().set(ControlMode.Velocity, inputVelocity);

    }

    public double metersPerSecondToTicks(double input){
        return input * Constants.METERS_PER_SECOND_TO_TICKS;
    }

    public double getModuleVelocity(SwerveModule module){
        return module.getDriveController().getDriveFalcon().getSelectedSensorVelocity();
    }

    public void resetSteerAngles(){
        m_frontLeftModule.getSteerController().resetAbsoluteAngle();
        m_frontRightModule.getSteerController().resetAbsoluteAngle();
        m_backLeftModule.getSteerController().resetAbsoluteAngle();
        m_backRightModule.getSteerController().resetAbsoluteAngle();
    }

    public void teleopInitLogSwerve(){
        logger.log(this, "TeleopInit SwerveValues", new SwerveModule[] {m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule});
    }
}