// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Treat like a static class.  No instantiation
    private Constants() {throw new UnsupportedOperationException();}

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.553;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.597;

    //
    // CAN IDs for the drivertrain motors and CANcoders
    //
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR    = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR    = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER  = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(148.8);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR     = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR     = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER   = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.5);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR     = 9;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR     = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER   = 7;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(134.7);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR     = 12;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR     = 11;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER   = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.6);

    //
    // CAN IDs for the ball collector and launching system
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

    // Vision constants for the ball camera
    public static double BALL_LOCK_ERROR       = 3.0;
    public static double BALL_CLOSE_ERROR      = 7.0;           // Closing in on acceptable error
    public static double BALL_ERROR            = 0.5;           // Allowed aiming error in degrees
    public static double BALL_CAMERA_HEIGHT    = 35.75;
    public static double BALL_CAMERA_ANGLE     = 30.0;
    public static double BALL_TARGET_HEIGHT    = 4.75;
    public static double BALL_ROTATE_KP        = 0.1; //0.15;           // Proportional constant for turret rotate speed
    public static double BALL_ROTATE_KI        = 0.0;
    public static double BALL_ROTATE_KD        = 0.01;   
    public static String LIMELIGHT_BALL        = "limelight-ball";

    public static double CLOSE_BALL_ROTATE_KP  = 0.6; //0.15;           // Proportional constant for turret rotate speed
    public static double CLOSE_BALL_ROTATE_KI  = 0.0;
    public static double CLOSE_BALL_ROTATE_KD  = 0.0;  

    // Common vision constants
    public static double MIN_TURN_SPEED = 0.8;

    // Color sensor
    public static int MIN_BALL_PROXIMITY = 300;
    public static int MAX_COLOR_CHECKS   = 10;

    // Main collector
    public static double COLLECT_PROCESSING_POWER =  0.25;
    public static double COLLECT_STAGING_POWER    =  0.3;
    public static double UNJAM_PROCESSING_POWER   = -0.2;
    public static double UNJAM_STAGING_POWER      = -0.2;
    public static double SHOOT_STAGING_POWER      =  1.0;

    // Collector arm
    public static int BALL_SET_POINT = -27500;   // -3025 is bottom
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


    //Constants for our starting position in autonomous
    public static final double POSITION_ERROR = 20; //error for checking what position we are in at the start of the match in degrees
    public static final double ANGLE_A = -82;
    public static final double ANGLE_B = -55;
    public static final double ANGLE_C = 24;
}
