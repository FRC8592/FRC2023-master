package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
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
    private SwerveModuleState[] states;   //possible hold the state of the swerve modules
    private SwerveDriveOdometry odometry; //Odometry object for swerve drive


    /**
     * The maximum voltage that will be delivered to the drive motors.
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     * TODO: 12.0 volts may be a little high.  Monitor battery voltage to determine if a lower voltage will work better
     */
    private static final double MAX_VOLTAGE = 12.0;

    // Measure the drivetrain's maximum velocity (m/s) or calculate the theoretical maximum.
    //
    // This formula is taken from the SDS swerve-template repository: https://github.com/SwerveDriveSpecialties/swerve-template
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    // The maximum angular velocity of the robot in radians per second.\
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
    public Drivetrain() {
        Mk4ModuleConfiguration swerveMotorConfig;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Create configuration object for motors.  We do this primarily for current limiting
        swerveMotorConfig = new Mk4ModuleConfiguration();
        swerveMotorConfig.setNominalVoltage(MAX_VOLTAGE);
        swerveMotorConfig.setDriveCurrentLimit(ConfigRun.MAX_SWERVE_DRIVE_CURRENT);
        swerveMotorConfig.setSteerCurrentLimit(ConfigRun.MAX_SWERVE_STEER_CURRENT);

        // Create motor objects
        //
        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // Motor configuration
            swerveMotorConfig,
            // This can either be L!, L2, L3 or L4
            Mk4SwerveModuleHelper.GearRatio.L2,
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
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
            swerveMotorConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
            swerveMotorConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
            swerveMotorConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        );

        this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());
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
        // FIXME Remove if you are using a Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

        // FIXME Uncomment if you are using a NavX
        //if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
         //   return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        //}

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }


    public double getAutoHeading() {
        return m_navx.getYaw();
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
        odometry.resetPosition(pose, new Rotation2d(0));
    }


    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        this.odometry.update(getGyroscopeRotation(), getSMState( m_frontLeftModule), getSMState(m_frontRightModule),getSMState(m_backLeftModule),
        getSMState(m_backRightModule));

    } 
    SwerveModuleState getSMState(SwerveModule mod){
        return new SwerveModuleState(mod.getDriveVelocity(), new Rotation2d(mod.getSteerAngle()));
    }
}