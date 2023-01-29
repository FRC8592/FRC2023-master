package frc.robot;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class FRCLogger {
    private boolean log;
    private String logFolder;

    /**
     * Initialize the logger
     * 
     * @param log       Whether to log or not when a class calls the log function
     * @param logFolder The folder to log to within RealOutputs; used to be
     *                  "CustomLogs"
     */
    public FRCLogger(boolean log, String logFolder) {
        this.log = log;
        this.logFolder = logFolder;
    }

    /**
     * Logs a single object. Acceptable datatypes are boolean, int, double,
     * String, boolean[], double[], and String[]. Datatypes that are
     * converted into one of the above automatically are:
     * 
     * <p>
     * int[]: Logs as a double[].
     * 
     * <p>
     * SwerveModule: Logs the steer angle and drive velocity, in that order, of
     * the SwerveModule. Steer angle is logged in both degrees and radians. Expects
     * input steer angle to be in degrees.
     * 
     * <p>
     * SwerveModule[]: Logs the steer angles and drive velocities, in that order,
     * of each SwerveModule to a double[]. Steer angle is logged in both degrees and
     * radians. Expects input steer angle to be in radians.
     * 
     * <p>
     * Pose2d: Logs the location X, location Y, and rotation, in that order, to
     * a double[]. Rotation is logged in degrees and radians; locations
     * X and Y are logged in meters and inches. Expects input locations X and Y
     * to be in meters.
     * 
     * <p>
     * Rotation2d: Logs rotation to a double. Rotation is logged in degrees and radians.
     * 
     * @param filePath An {@code Object} containing the "folder" that the logged data is
     *                 stored in. Use a {@code String} to specify a custom path within the
     *                 log folder, or input {@code this} to automatically name and
     *                 generate the path.
     * @param name     A {@code String} containing the name of the logged data. Usually the
     *                 name of the function or variable that is being logged.
     * @param data     An {@code Object} of one of the acceptable datatypes listed above
     *                 containing the data to log.
     */
    public void log(Object filePath, String name, Object data) {
        if (log) {
            String path;
            if (filePath.getClass().getSimpleName().equals("String")) {
                path = (String) filePath;
            } else {
                if(filePath.getClass().getSimpleName().equals("Vision")){
                    Vision vision = (Vision) filePath;
                    path = vision.getVisionName();
                }
                else{
                    path = filePath.getClass().getSimpleName();
                }
            }

            // Check for the "data" object being one of the supported datatypes and log it.
            if (data.getClass().getSimpleName().equals("boolean"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean) data);
            if (data.getClass().getSimpleName().equals("int"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (int) data);
            if (data.getClass().getSimpleName().equals("double"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double) data);
            if (data.getClass().getSimpleName().equals("String"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String) data);
            if (data.getClass().getSimpleName().equals("boolean[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean[]) data);
            if (data.getClass().getSimpleName().equals("int[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data); // double[] here because int[] isn't suposrted by AK
            if (data.getClass().getSimpleName().equals("double[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data);
            if (data.getClass().getSimpleName().equals("String[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String[]) data);

            // Now the if-statements that convert datatypes that Advantage* doesn't
            // support into something it does.

            if (data instanceof SwerveModule) {
                SwerveModule swerveModule = (SwerveModule) data;
                double[] resultDegrees = new double[2];
                resultDegrees[0] = Math.toDegrees(swerveModule.getSteerAngle());
                resultDegrees[1] = swerveModule.getDriveVelocity();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name,
                        resultDegrees);
            }
            if (data instanceof SwerveModule[]) {
                SwerveModule[] swerveModules = (SwerveModule[]) data;
                double[] resultDegrees = new double[2 * swerveModules.length];
                for (int i = 0; i < swerveModules.length; i++) {
                    resultDegrees[i * 2] = Math.toDegrees(swerveModules[i].getSteerAngle());
                    resultDegrees[i * 2 + 1] = swerveModules[i].getDriveVelocity();
                }
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name,
                        resultDegrees);
            }
            if (data instanceof Pose2d) {
                Pose2d pose = (Pose2d) data;
                double[] result = new double[3];
                result[0] = pose.getX() * 39.3701;
                result[1] = pose.getY() * 39.3701;
                result[2] = pose.getRotation().getDegrees();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name,
                        result);
            }
            if (data instanceof Rotation2d) {
                Rotation2d rotation = (Rotation2d) data;
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Degrees",
                        rotation.getDegrees());
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Radians",
                        rotation.getRadians());
            }

            //2023 objects

            if (data instanceof Pose3d) {
                Pose3d pose = (Pose3d) data;
                double[] result = new double[7];
                double[] resultXYZ = new double[6];
                result[0] = pose.getX();
                result[1] = pose.getY();
                result[2] = pose.getZ();
                result[3] = pose.getRotation().getQuaternion().getW();
                result[4] = pose.getRotation().getQuaternion().getX();
                result[5] = pose.getRotation().getQuaternion().getY();
                result[6] = pose.getRotation().getQuaternion().getZ();
                resultXYZ[0] = pose.getX();
                resultXYZ[1] = pose.getY();
                resultXYZ[2] = pose.getZ();
                resultXYZ[3] = pose.getRotation().getX();
                resultXYZ[4] = pose.getRotation().getY();
                resultXYZ[5] = pose.getRotation().getZ();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Quaternion",
                        result);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/XYZ",
                        resultXYZ);
            }
            if (data instanceof Rotation3d) {
                Rotation3d rotation = (Rotation3d) data;
                double[] resultQ = new double[4];
                double[] resultXYZ = new double[3];
                resultXYZ[0] = rotation.getX();
                resultXYZ[1] = rotation.getY();
                resultXYZ[2] = rotation.getZ();
                resultQ[0] = rotation.getQuaternion().getW();
                resultQ[1] = rotation.getQuaternion().getX();
                resultQ[2] = rotation.getQuaternion().getY();
                resultQ[3] = rotation.getQuaternion().getZ();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Quaternion",
                        resultQ);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/XYZ",
                        resultXYZ);
            }
            if (data instanceof Quaternion) {
                Quaternion rotation = (Quaternion) data;
                double[] resultQ = new double[4];
                resultQ[0] = rotation.getW();
                resultQ[1] = rotation.getX();
                resultQ[2] = rotation.getY();
                resultQ[3] = rotation.getZ();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Quaternion",
                        resultQ);
            }
            
        }
    }
}