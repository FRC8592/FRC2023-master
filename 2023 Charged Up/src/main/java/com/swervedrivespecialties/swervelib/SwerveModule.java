package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setDriveVoltage(double voltage);

    void setSteerAngle(double voltage);

    DriveController getDriveController();
}
