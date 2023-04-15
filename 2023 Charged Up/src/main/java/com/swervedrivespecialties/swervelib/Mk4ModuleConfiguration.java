package com.swervedrivespecialties.swervelib;

import java.util.Objects;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4ModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    // throttle motor PID values
    public double throttlekP = Double.NaN;
    public double throttlekI = Double.NaN;
    public double throttlekD = Double.NaN;

    // steer motor PID values
    public double steerkP = Double.NaN;
    public double steerkI = Double.NaN;
    public double steerkD = Double.NaN;

    /**
     * 
     * @param kP The proportional value to set to the throttle motor
     * @param kI The integral value to set to the throttle motor
     * @param kD The derivative value to set to the throttle motor
     */
    public void setThrottlePID(double kP, double kI, double kD){
        this.throttlekP = kP;
        this.throttlekI = kI;
        this.throttlekD = kD;
    }
    
    /**
     * 
     * @param kP The proportional value to set to the steer motor
     * @param kI The integral value to set to the steer motor
     * @param kD The derivative value to set to the steer motor
     */
    public void setSteerPID(double kP, double kI, double kD){
        this.steerkP = kP;
        this.steerkI = kI;
        this.steerkD = kD;
    }
    

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                '}';
    }
}
