 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * gloDRIVE_TOy (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Treat like a static class.  No instantiation
    private Constants() {throw new UnsupportedOperationException();}
    // public static final double JOYSTICK_SCALE_FACTOR = 0.5;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
    public static final double kWheelCircumference = 4 * Math.PI;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62865;

    //
    // CAN IDs for the drivertrain motors and CANcoders
    //
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR    = 2; // Named Green in Electronics
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR    = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER  = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(90.7);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR     = 8; // Named Black in Electronics
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR     = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER   = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(66.3);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR     = 6; // Named Orange in Electronics
    public static final int BACK_LEFT_MODULE_STEER_MOTOR     = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER   = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(136.7);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR     = 4; // Named White in Electronics
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR     = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER   = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(285.0);


    public static final double LEVEL_PITCH = 3.0;
    public static final double PITCH_MULTIPLIER = 0.01612903;
    //
    // CAN IDs for intake hardware (NEEDS SETTING)
    //
    public static final int WRIST_ID    = 17;
    public static final int ROLLER_ID    = 18;
    public static final int BEAM_BREAK_CONE_ID      = 3;
    public static final int BEAM_BREAK_CUBE_ID      = 2;
    public static final int ROLLER_MAX_CURRENT_AMPS = 60;
    public static final int WRIST_MAX_CURRENT_AMPS = 30;
    public static final double ROLLER_KP = 0.00015; // 0.00025
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.01;
    public static final double ROLLER_KF = 0.0;
    public static final double WRIST_KP = 0.00025;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = -0.00003;
    public static final double WRIST_KF = 0.0;
    public static final double ROLLER_GEAR_RATIO = 1.0 / 8.0;
    public static final double WRIST_GEAR_RATIO = 1.0 / 64.0;
    public static final int ROLLER_PID_SLOT = 0;
    public static final int WRIST_PID_SLOT = 0;
    public static final double ROLLER_MAX_VELOCITY = 5676.0 / ROLLER_GEAR_RATIO;
    public static final double ROLLER_MAX_ACCELERATION = ROLLER_MAX_VELOCITY; // Rotations per minute per second
    public static final double ROLLER_CUBE_INTAKE_CURRENT_THRESHOLD = 8.0; // roller cube limit (for piece detection)
    public static final double WRIST_MAX_ACCELERATION = 10000.0; // Rotations per minute per second
    public static final double WRIST_MAX_VELOCITY = 6000.0; // Rotations per minute
    public static final double WRIST_INTAKE_ROTATIONS = 34.75;
    public static final double WRIST_INTAKE_TELEOP_ROTATIONS = 33.5;
    public static final double WRIST_STOWED_ROTATIONS = 0;
    public static final double WRIST_SCORING_ROTATIONS = 34.25;
    public static final double WRIST_MAX_ROTATIONS = 33.0;

    //
    // Constants and IDs for elevator hardware (NEEDS SETTING)
    //
    public static final int ELEVATOR_LIFT_MOTOR_ID   = 16;
    public static final int ELEVATOR_TILT_MOTOR_ID   = 15;
    public static final double LIFT_GEARBOX_RATIO    = 1.0 / 16.0;
    public static final double TILT_GEARBOX_RATIO    = 1.0 / 80.0 * 16.0 / 32.0;
    public static final double TILT_MAX_ROTATIONS = -23.5; // -23.5
    public static final double LIFT_MAX_ROTATIONS = -80;
    public static final double LIFT_STOWED_ROTATIONS = 0;
    public static final double LIFT_MAX_INCHES = -63;
    public static final double TILT_THRESHOLD_TO_LIFT = -18;
    public static final double LIFT_THRESHOLD_TO_STOW = -10;


    //
    // IDs for claw pneumatics (NEEDS SETTING)
    //
    public static final int PNEUMATIC_MODULE_ID = 14;
    public static final int CLAW_PNEUMATIC_CHANNEL = 15;
    public static final int MIN_COMPRESSOR_PSI = 60;
    public static final int MAX_COMPRESSOR_PSI = 100;


    //
    // CAN IDs for the DRIVE_TO collector and launching system
    //
    public static final int newFlywheelLeft      = 22;
    public static final int newFlywheelRight     = 20;
    public static final int newFlywheelStaging   = 21;
    public static final int newFlywheelCollector = 23;
    public static final int COLLECTOR_ARM_CAN    = 24;

    // CAN IDs for the lift motors
    public static final int LIFT_RIGHT_CAN = 25;
    public static final int LIFT_LEFT_CAN  = 26;

    // Other CAN IDs
    public static final int PDH_CAN = 1;

    // DIO Ports
    public static final int LINE_BREAK_TOP_SENSOR_PORT    = 0;
    public static final int LINE_BREAK_BOTTOM_SENSOR_PORT = 1;
    public static final int COLLECTOR_ARM_LIMIT_SWITCH    = 3;

    // Constants for controlling the shooter flywheel
    public static double FLYWHEEL_VOLTAGE = 11;                // Maximum controller voltage for voltage compensation
    public static double FLYWHEEL_FAST_P = 0.6;               //0.35;                   // Starting value.  Needs tuning
    public static double FLYWHEEL_FAST_I = 0.0;               // Starting value.  Needs tuning
    public static double FLYWHEEL_FAST_D = 60.00;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_FAST_F = 0.054;                   // Starting value.  Needs tuning
   
    public static double FLYWHEEL_SLOW_P = 0.17;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_SLOW_I = 0.0;               // Starting value.  Needs tuning
    public static double FLYWHEEL_SLOW_D = 0.00;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_SLOW_F = 0.056;                   // Starting value.  Needs tuning
    
    public static int    FLYWHEEL_LOCK_RANGE = 10;              // Lock in a 10' shooting range
    public static double STARTING_FLYWHEEL_SPEED = 2300;
    public static double REJECT_FLYWHEEL_SPEED   = 500;
    public static double RPM_TO_TICKS_MS = 2048.0 / 600.0;     // Conversion factor for rotational velocity (RPM to ticks per 100ms)
    public static double RPM_MAX_ERROR   = 5;                  // Allowed RPM error for flywheel

    // Vision constants for the ring camera
    public static double RING_LOCK_ERROR       = 2.0;           // Angular error allowed for targetting
    public static double RING_CLOSE_ERROR      = 6.0;           // Closing in on acceptable error
    public static double TURRET_ERROR          = 0.5;           // Allowed aiming error in degrees
    public static double RING_CAMERA_HEIGHT    = 36.75;         // Limelight height above ground (inches)
    public static double RING_CAMERA_ANGLE     = 30.0;          // Limelight camera angle above horizontal (degrees)
    public static double RING_TARGET_HEIGHT    = 104.0;         // Center of target above ground (inches)
    public static double TURRET_ROTATE_KP      = 0.185;         // Proportional constant for rotate speed
    public static double TURRET_ROTATE_KI      = 0;             //0.000001;
    public static double TURRET_ROTATE_KD      = 0.022; 
    public static String LIMELIGHT_RING        = "limelight-ring";

    // Vision constants for the DRIVE_TO camera
    public static final double DRIVE_TO_LOCK_ERROR       = 3.0;
    public static final double DRIVE_TO_CLOSE_ERROR      = 7.0;           // Closing in on acceptable error
    public static double DRIVE_TO_ERROR            = 0.5;           // Allowed aiming error in degrees
    public static double DRIVE_TO_CAMERA_HEIGHT    = 35.75;
    public static double DRIVE_TO_CAMERA_ANGLE     = -30.0;
    public static double DRIVE_TO_TARGET_HEIGHT    = 4.75;
    public static double DRIVE_TO_ROTATE_KP        = -0.1; //0.15;           // Proportional constant for turret rotate speed
    public static double DRIVE_TO_ROTATE_KI        = 0.0;
    public static double DRIVE_TO_ROTATE_KD        = 0.0;   
    public static String LIMELIGHT_DRIVE_TO        = "limelight-DRIVE_TO";
    public static String LIMELIGHT_VISION       = "limelight-vision";

    public static double CLOSE_DRIVE_TO_ROTATE_KP  = 0.6; //0.15;           // Proportional constant for turret rotate speed
    public static double CLOSE_DRIVE_TO_ROTATE_KI  = 0.0;
    public static double CLOSE_DRIVE_TO_ROTATE_KD  = 0.0;  

    // Vision constants for the TURN_TO camera
    public static double TURN_TO_ROTATE_KP        = -0.1; //0.15;           // Proportional constant for turret rotate speed
    public static double TURN_TO_ROTATE_KI        = 0.0;
    public static double TURN_TO_ROTATE_KD        = 0.0;   


    // Common vision constants
    public static double MIN_TURN_SPEED = 0.8;

    // Color sensor
    public static int MIN_DRIVE_TO_PROXIMITY = 300;
    public static int MAX_COLOR_CHECKS   = 10;

    // Main collector
    public static double COLLECT_PROCESSING_POWER =  0.25;
    public static double COLLECT_STAGING_POWER    =  0.3;
    public static double UNJAM_PROCESSING_POWER   = -0.2;
    public static double UNJAM_STAGING_POWER      = -0.2;
    public static double SHOOT_STAGING_POWER      =  1.0;

    // Collector arm
    public static int DRIVE_TO_SET_POINT = -27500;   // -3025 is bottom
    public static double ARM_UP_P = 0.12;
    public static double ARM_UP_I = 0.0001;
    public static double ARM_UP_D = 15.0;
    public static double ARM_UP_F = 0.0;

    public static double ARM_DOWN_P = 0.12;
    public static double ARM_DOWN_I = 0.0;
    public static double ARM_DOWN_D = 10.0;
    public static double ARM_DOWN_F = 0.0;

    public static int    ARM_MM_SMOOTHING   = 1;
    public static double ARM_MM_CRUISE_VELO = 16000;
    public static double ARM_MM_ACCEL       = 15000;
    public static double ARM_DEADBAND       = 0.001;  // Set very small.  Default is 0.04
    public static double ARM_STEADY_POWER   = 0.10;
    public static int    ARM_TICKS_180      = 42624;

    // Limit collector arm current
    public static SupplyCurrentLimitConfiguration ARM_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 10, 40, 6.0);

    // Constants for the lift
    public static double LIFT_VOLTAGE = 11;     // Maximum controller voltage for voltage compensationble 
    public static SupplyCurrentLimitConfiguration LIFT_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 12, 20, 0.5);

    public static int LIFT_TOP_POINT = 0;   // Need to measure top
    public static double LIFT_UP_P = 0.1;   // Starting value.  Needs tuning
    public static double LIFT_UP_I = 0.0;   // Starting value.  Needs tuning
    public static double LIFT_UP_D = 0.0;   // Starting value.  Needs tuning
    public static double LIFT_UP_F = 0.0;   // Starting value.  Needs tuning

    public static double LIFT_DOWN_P = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_I = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_D = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_F = 0.0;     // Starting value.  Needs tuning

    public static int    LIFT_MM_SMOOTHING   = 1;
    public static double LIFT_MM_CRUISE_VELO = 150000;
    public static double LIFT_MM_ACCEL       = 600000;
    public static double LIFT_DEADBAND       = 0.001;        // Set very small.  Default is 0.04
    public static double LIFT_MAX_POWER      = 0.5;          // Absolute max power allowed for lift motors
    public static double LIFT_STEADY_POWER   = 0.10;  
    public static double LIFT_PARK_POWER     = 0.08;         // Power to drive arms to parked position
    public static int    LIFT_TICKS_180      = 4736;
    public static double LIFT_PARKED_CURRENT = 10;           // amount of current to check if arms in parked position
    public static double LIFT_MAX_POSITION   = 0.0;          // Max number of ticks, 27(gear) * 2048() * 6 (3" per rotation, 18" fully extended)
    public static double LIFT_RIGHT_MIN_POSITION = -395000.0;
    public static double LIFT_LEFT_MIN_POSITION  = -395000.0;
    public static double LIFT_CHANGE_POSITION= 3500;         // add or subtract # of ticks to move arms
    public static double LIFT_FEED_FORWARD   = 0.18;   

    // Table for flywheel speeds.  Each entry represents 12" of distance from reflectors
    public static double RANGE_TABLE[] = {
        2110,  // 0 ft
        2110,  // 1 ft
        2110,  // 2 ft
        2110,  // 3 ft
        2110,  // 4 ft
        2110,  // 5 ft
        2110,  // 6 ft
        2130,  // 7 ft
        2150,  // 8 ft
        2200,  // 9 ft 
        2300,  // 10 ft
        2390,  // 11 ft
        2490,  // 12 ft
        2615,  // 13 ft
        2750,  // 14 ft 
        2855,  // 15 ft 
        3055,  // 16 ft 
        3120,  // 17 ft
        3280,  // 18 ft
        3400,  // 19 ft
        3400}; // 20 ft

    public static double FLYWHEEL_LIMIT = 3280;

    // Limelight LED modes
    public static enum LIMELIGHT_LIGHT {PIPELINE_MODE, FORCE_OFF, FORCE_BLINK, FORCE_ON}

    // LED Constants
    public static final int LED_LENGTH = 8;
    public static final double MINIMUM_VOLTAGE = 9.0;
    public static final int PULSE_METHOD_SPEED = 5;
    public static final int PULSE_SIZE = 2; 
    public static final int PULSE_GAP = 5;


    //Constants for our starting position in autonomous
    public static final double POSITION_ERROR = 20; //error for checking what position we are in at the start of the match in degrees
    public static final double ANGLE_A = -82;
    public static final double ANGLE_B = -55;
    public static final double ANGLE_C = 24;

    public static final int CUBE_PIPELINE = 0;
    public static final int CONE_PIPELINE = 1;
    public static final int APRILTAG_PIPELINE = 2;
    public static final int RETROTAPE_PIPELINE = 4;

    /*
     * 2048 - Total num ticks in Falcon 500
     * 6.75 - MK4i module gear ratio
     * 60 - Seconds?
     * 
     */
    public static final double METERS_PER_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));
    public static final double WHEEL_LOCK_RADIANS = 0.785398;

    public static final double TURN_TO_kP = 0.0001;
    public static final double TURN_TO_kI = 0;
    public static final double TURN_TO_kD = 0.0;

    // Conversion constants
    public static final double INCHES_TO_METERS = 1.0 / 39.37;

    public static final int SUBSTATION_CONE_PIPELINE = 0;
    public static final double SUBSTATION_OFFSET = 8.0;
    public static final double SUBSTATION_ACCEPTANCE_RADIUS = 5.0;
    public static String LIMELIGHT_REAR       = "limelight-rear";

    public static final double MAX_JOYSTICK_DECELERATION = 0.075; //0.18
}

