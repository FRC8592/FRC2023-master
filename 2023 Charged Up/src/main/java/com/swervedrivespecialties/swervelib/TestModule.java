package com.swervedrivespecialties.swervelib;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

public class TestModule {
    private SwerveModule module;
    
    public TestModule() {
        module = Mk4iSwerveModuleHelper.createFalcon500(GearRatio.L2, 0, 0, 0, 0);
    }

    public void test() {
        module.setDriveVoltage(0);
        module.getDriveController().getDriveFalcon();
    }
}
