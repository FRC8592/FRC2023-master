package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;


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
    private PIDController turnToPID = new PIDController(Constants.TURN_TO_kP, TURN_TO_kI, TURN_TO_kD);
    private FRCLogger logger;

    private PIDController turnPID;

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
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5; //4.5
    // 6380.0 / 60.0 *
    //     SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
    //     SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    // The maximum angular velocity of the robot in radians per second.
    //
    // This calculated value could be replaced with a measured value.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / //6.0
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
        this.m_navx.enableLogging(false);
        Mk4ModuleConfiguration swerveMotorConfig;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Create configuration object for motors.  We do this primarily for current limiting
        swerveMotorConfig = new Mk4ModuleConfiguration();
        swerveMotorConfig.setNominalVoltage(MAX_VOLTAGE);
        swerveMotorConfig.setDriveCurrentLimit(ConfigRun.MAX_SWERVE_DRIVE_TELEOP_CURRENT);
        swerveMotorConfig.setSteerCurrentLimit(ConfigRun.MAX_SWERVE_STEER_CURRENT);

        //set PID constants
        swerveMotorConfig.setThrottlePID(0.02, 0, 0.01); //0.02, 0, 0.01
        swerveMotorConfig.setSteerPID(0.2, 0.0, 0.1);
        
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
        turnToPID.enableContinuousInput(-180, 180);
        turnToPID.setTolerance(10.0);

        turnPID = new PIDController(0.05, 0, 0);
        turnPID.setTolerance(5);
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
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        // if (DriverStation.getAlliance() == Alliance.Red) {
        //     return Rotation2d.fromDegrees(180-m_navx.getYaw());
        // }
        // SmartDashboard.putNumber("GYRO ROTATION", -m_navx.getYaw());
        return Rotation2d.fromDegrees(-m_navx.getYaw());
    }

    public double getPitch(){
        return m_navx.getPitch();
    }

    public double getRoll(){
        return m_navx.getRoll();
    }


    public double getAutoHeading() {
        return m_navx.getYaw();
    }
    

    public boolean isGyroscopeRotating(){
        return m_navx.isRotating();
    }

    public double getYaw(){
        // double yaw = m_navx.getYaw();
        // if (yaw < 0) {
        //     yaw += 360;
        // }
        return m_navx.getYaw();
    }

    public Pose2d getCurrentPos(){
        Pose2d pos = odometry.getPoseMeters();
        // SmartDashboard.putNumber("Drive X", pos.getX()); //meters to inches
        // SmartDashboard.putNumber("Drive Y", pos.getY());

        // SmartDashboard.putNumber("Drive X (in)", pos.getX() * 39.3701); //meters to inches
        // SmartDashboard.putNumber("Drive Y (in)", pos.getY()  * 39.3701 );
        // SmartDashboard.putNumber("Drive Yaw (deg)", pos.getRotation().getDegrees());
        return pos;
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(new Rotation2d(0), new SwerveModulePosition[]  {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, pose);
    }
    
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        //SmartDashboard.putNumber("Chassis Speeds X", chassisSpeeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("Chassis Speeds Y", chassisSpeeds.vyMetersPerSecond);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        double frontLeftVelo = states[0].speedMetersPerSecond;
        double frontRightVelo = states[1].speedMetersPerSecond;
        double backLeftVelo = states[2].speedMetersPerSecond;
        double backRightVelo = states[3].speedMetersPerSecond;

        setModule(m_frontLeftModule, states[0].angle.getRadians(), metersPerSecondToTicks(frontLeftVelo));
        setModule(m_frontRightModule, states[1].angle.getRadians(), metersPerSecondToTicks(frontRightVelo));
        setModule(m_backLeftModule, states[2].angle.getRadians(), metersPerSecondToTicks(backLeftVelo));
        setModule(m_backRightModule, states[3].angle.getRadians(), metersPerSecondToTicks(backRightVelo));
        
        this.odometry.update(
            getGyroscopeRotation(), 
            new SwerveModulePosition[] {
                getSMPosition(m_frontLeftModule), 
                getSMPosition(m_frontRightModule),
                getSMPosition(m_backLeftModule),
                getSMPosition(m_backRightModule)
            }
        );


        /*
        * Swerve Module States Array
        * 0 - Front Left Azimuth
        * 1 - Front Left Veolcity
        *
        * 2 - Front Right Azimuth
        * 3 - Front Right Velocity
        *
        * 4 - Back Left Azimuth
        * 5 - Back Left Veolcity
        * 
        * 6 - Back Right Azimuth
        * 7 - Back Right Velocity
        */
        logger.log(this, "SwerveModuleStates", new SwerveModule[] {m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule});
        
        /*
         * Swerve Module Current and Velocities array
         * 0 - Front Left
         * 1 - Front Right
         * 2 - Back Left
         * 3 - Back Right
         */
        logger.log(this, "SwerveThrottleCurrents", getThrottleAppliedCurrent());
        logger.log(this, "SwerveModuleDesiredVelocity", new double[] {frontLeftVelo, frontRightVelo, backLeftVelo, backRightVelo});
        logger.log(this, "SwerveModuleActualVelocity", getThrottleAppliedVelocity());
        // logger.log(this, "CANCoder Values", new double[] {m_frontLeftModule.getSteerAngle(), m_frontRightModule.getSteerAngle(), })
    } 

    public void resetEncoder(){
        m_frontLeftModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_frontRightModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_backLeftModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
        m_backRightModule.getDriveController().getDriveFalcon().setSelectedSensorPosition(0);
    }
    /**
     * Turn to an absolute angle
     * @param targetDegrees the absolute angle to turn to
     * @return the speed the robot needs to turn, based on the PID controller
     */
    public double turnToAngle(double targetDegrees){
        double yaw = getYaw();
        // targetDegrees = 180 - targetDegrees;

        // if (targetDegrees > 180) {
        //     targetDegrees -= 360;
        // }

        // targetDegrees = 180 - targetDegrees;
        // targetDegrees *= -1;

        // double targetDifference = yaw - targetDegrees;
        double turn = turnPID.calculate(0, getErrorAngle(getCurrentPos(), new Pose2d(0, 0, Rotation2d.fromDegrees(targetDegrees))));

        // SmartDashboard.putNumber("Current Yaw", yaw);
        // SmartDashboard.putNumber("Target Degrees", targetDegrees);
        // SmartDashboard.putNumber("Target Difference", yaw - targetDegrees);
        // SmartDashboard.putNumber("TurnTo PID", -turn);

        // return -turnToPID.calculate(yaw, targetDegrees);
        return -turn;
    }

    private double getErrorAngle(Pose2d robot, Pose2d goal){
        /*** Computation for currect rotate errors into waypoint ****/
       double goalAngle = goal.getRotation().getDegrees();
       double curAngle = robot.getRotation().getDegrees();
       if (curAngle < 0) {
            curAngle += 360;
       }
       double errorAngle = 0; 
       //we only use angles between 0 and 2 PI so convert the angles to that range.
       if(goalAngle < 0){
                   goalAngle += 360;    
       }
       // find shortest angle difference error angle should allways be > -PI and <= PI
       errorAngle = goalAngle - curAngle;   
       if(errorAngle > 180){
           errorAngle -= 360;
       } else if(errorAngle <= -180){
           errorAngle += 360;
       } 
       return errorAngle;
   }

    public void getSwervePositions() {
        // SmartDashboard.putNumber("Front Left Posiiton", m_frontLeftModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        // SmartDashboard.putNumber("Front Right Posiiton", m_frontRightModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        // SmartDashboard.putNumber("Back Left Posiiton", m_backLeftModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
        // SmartDashboard.putNumber("Back Right Posiiton", m_backRightModule.getDriveController().getDriveFalcon().getSelectedSensorPosition()*kFalconTicksToMeters);
    }

    private SwerveModulePosition getSMPosition(SwerveModule mod){
        return new SwerveModulePosition(mod.getDriveController().getDriveFalcon().getSelectedSensorPosition()/4096.0/Constants.kWheelCircumference, new Rotation2d(mod.getSteerAngle()));
    }

    public void setDriveVelocity(double inputVelocity, SwerveModule module){
        module.getDriveController().getDriveFalcon().set(ControlMode.Velocity, inputVelocity);

    }

    public double metersPerSecondToTicks(double input){
        return input * Constants.METERS_PER_SECOND_TO_TICKS;
    }
    public double ticksToMetersPerSecond(double input){
        return input / Constants.METERS_PER_SECOND_TO_TICKS;
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

    public void setModule(SwerveModule module, double steerAngle, double velocityMetersPerSecond){
        double velocityToApply;
        if (module.setModuleSteerAngle(steerAngle)){
            velocityToApply = -velocityMetersPerSecond;
        }else{
            velocityToApply = velocityMetersPerSecond;
        }
        // SmartDashboard.putNumber("Velocity to Apply", velocityToApply);
        setDriveVelocity(velocityToApply, module);
    }
    /* WHEEL LOCK MODE FOR AUTOPARK - Liam M */
    public void setWheelLock(){
        // m_frontLeftModule.setSteerAngle(Constants.WHEEL_LOCK_RADIANS);
        // m_frontRightModule.setSteerAngle(-Constants.WHEEL_LOCK_RADIANS);
        // m_backLeftModule.setSteerAngle(-Constants.WHEEL_LOCK_RADIANS);
        // m_backRightModule.setSteerAngle(Constants.WHEEL_LOCK_RADIANS);

        setModule(m_frontLeftModule, Constants.WHEEL_LOCK_RADIANS, 0);
        setModule(m_frontRightModule, -Constants.WHEEL_LOCK_RADIANS, 0);
        setModule(m_backLeftModule, -Constants.WHEEL_LOCK_RADIANS, 0);
        setModule(m_backRightModule, Constants.WHEEL_LOCK_RADIANS, 0);
    }
    /********** END WHEEL LOCK CODE ***********/

    /* CURRENT LIMIT CODE - Liam M */
    private void setThrottleCurrentLimit(double currentLimit){

        m_frontLeftModule.getDriveController().getDriveFalcon().configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, 0, 0));
        m_frontRightModule.getDriveController().getDriveFalcon().configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, 0, 0));
        m_backLeftModule.getDriveController().getDriveFalcon().configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, 0, 0));
        m_backRightModule.getDriveController().getDriveFalcon().configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, 0, 0));
    }

    public void setTeleopCurrentLimit(){
        setThrottleCurrentLimit(ConfigRun.MAX_SWERVE_DRIVE_TELEOP_CURRENT);
    }
    public void setAutoCurrentLimit(){
        setThrottleCurrentLimit(ConfigRun.MAX_SWERVE_DRIVE_AUTO_CURRENT);
    }
    /********END CURRENT LIMIT CODE**********/

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public double[] getThrottleAppliedCurrent(){
        double frontLeftCurrent = m_frontLeftModule.getDriveController().getDriveFalcon().getSupplyCurrent();
        double frontRightCurrent = m_frontRightModule.getDriveController().getDriveFalcon().getSupplyCurrent();
        double backLeftCurrent = m_backLeftModule.getDriveController().getDriveFalcon().getSupplyCurrent();
        double backRightCurrent = m_backRightModule.getDriveController().getDriveFalcon().getSupplyCurrent();

        return new double[] {frontLeftCurrent, frontRightCurrent, backLeftCurrent, backRightCurrent};
    }

    public double[] getThrottleAppliedVelocity(){
        double frontLeftVelo = ticksToMetersPerSecond(m_frontLeftModule.getDriveController().getDriveFalcon().getSelectedSensorVelocity());
        double frontRightVelo = ticksToMetersPerSecond(m_frontRightModule.getDriveController().getDriveFalcon().getSelectedSensorVelocity());
        double backLeftVelo = ticksToMetersPerSecond(m_backLeftModule.getDriveController().getDriveFalcon().getSelectedSensorVelocity());
        double backRightVelo = ticksToMetersPerSecond(m_backRightModule.getDriveController().getDriveFalcon().getSelectedSensorVelocity());

        return new double[] {Math.abs(frontLeftVelo), Math.abs(frontRightVelo), Math.abs(backLeftVelo), Math.abs(backRightVelo)};
    }
}