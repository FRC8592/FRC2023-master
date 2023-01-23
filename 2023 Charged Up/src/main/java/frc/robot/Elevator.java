package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Elevator {
    private CANSparkMax elevatorArmMotor;
    private CANSparkMax elevatorAngleMotor;
    private RelativeEncoder armEncoder;
    private RelativeEncoder angleEncoder;

    private PIDController upPIDController;
    private PIDController downPIDController;
    private PIDController armAnglePIDController;

    public enum ScoreStates {
        STOWED(Constants.STOWED_HEIGHT),
        LOW(Constants.LOW_HEIGHT),
        MID(Constants.MID_HEIGHT),
        HIGH(Constants.HIGH_HEIGHT);

        double height;
        ScoreStates(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public Elevator() {
        elevatorArmMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        armEncoder = elevatorArmMotor.getEncoder();
        elevatorAngleMotor = new CANSparkMax(Constants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleEncoder = elevatorAngleMotor.getEncoder();
        //PID constants WIP
        upPIDController = new PIDController(0.01, 0, 0);
        downPIDController = new PIDController(0.01, 0, 0);
        armAnglePIDController = new PIDController(0.01, 0, 0);
    }

    public ScoreStates upOneLevel (ScoreStates current) {
        switch(current) {
            case LOW:
                return ScoreStates.MID;
            case MID:
                return ScoreStates.HIGH;
        }
        return current;
    }

    public ScoreStates downOneLevel (ScoreStates current) {
        switch(current) {
            case MID:
                return ScoreStates.LOW;
            case HIGH:
                return ScoreStates.MID;
        }
        return current;
    }

    public void liftArm() {
        // ENCODER POSITION UNITS NEED TO BE CONVERTED (?)

        elevatorAngleMotor.set(armAnglePIDController.calculate(angleEncoder.getPosition(), Constants.ARM_ANGLE));
    }

    public void lowerArm() {
        // ENCODER POSITION UNITS NEED TO BE CONVERTED (?)

        elevatorAngleMotor.set(armAnglePIDController.calculate(angleEncoder.getPosition(), 0));
    }

    // For all:
    //  Put in brake mode
    //  Hard cap drivetrain accel when extended past mid (extension point subject to change)
    //      these two WIP in robot.java
    // Bring claw to desired position
    public void set(ScoreStates objective) {
        //ENCODER POSITION UNITS NEED TO BE CONVERTED (?)

        if(objective.getHeight() > armEncoder.getPosition()) {
            elevatorArmMotor.set(upPIDController.calculate(armEncoder.getPosition(), objective.getHeight()));
        } else {
            elevatorArmMotor.set(downPIDController.calculate(armEncoder.getPosition(), objective.getHeight()));
        }
    }

    // Call on init (robot.java?), will reset position and make sure elevator is in frame perimeter
    public void reset() {
        set(ScoreStates.STOWED);
        armEncoder.setPosition(0.0);
    }

    public double getEncoderPosition() {
        return armEncoder.getPosition();
    }
}
