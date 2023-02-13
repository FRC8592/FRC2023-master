package frc.robot;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.kinematics.*;
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
     * @param log       Whether log() does anything when called
     * @param logFolder The folder to log to within RealOutputs; used to be
     *                  "CustomLogs"
     */
    public FRCLogger(boolean log, String logFolder) {
        this.log = log;
        this.logFolder = logFolder;
    }

    /**
     * Initialize the logger with the the log folder set to {@code "CustomLogs"}
     * 
     * @param log Whether log() does anything when called
     */
    public FRCLogger(boolean log) {
        this.log = log;
        setDefaultLogFolder();
    }

    /**
     * Initialize the logger with logging enabled.
     * 
     * @param logFolder The folder to log to within RealOutputs
     */
    public FRCLogger(String logFolder) {
        this.logFolder = logFolder;
        enableLogging();
    }

    /**
     * Initialize the logger with default settings; log folder set to
     * {@code "CustomLogs"} and logging enabled.
     */
    public FRCLogger() {
        enableLogging();
        setDefaultLogFolder();
    }

    /**
     * Logs a single object. Acceptable objects are {@code boolean}, {@code int},
     * {@code double}, {@code String}, {@code boolean[]}, {@code double[]}, and
     * {@code String[]}. In no particular order, objects that are converted into one
     * of the above automatically are:
     * 
     * <p>
     * {@code int[]}: Logs as a {@code double[]}.
     * 
     * <p>
     * {@code Rotation2d}: Logs rotation to a {@code double}. Rotation is logged in
     * degrees and radians.
     * 
     * <p>
     * {@code SwerveModule}: Logs the steer angle and drive velocity of the
     * SwerveModule to a {@code double[]}.
     * 
     * <p>
     * {@code SwerveModule[]}: Logs the steer angles and drive velocities of each
     * SwerveModule to a {@code double[]}.
     * 
     * <p>
     * {@code Pose2d}: Logs the location and rotation to a {@code double[]}.
     * Rotation is logged in degrees.
     * 
     * <p>
     * {@code Pose3d}: Logs the location and rotation of the Pose3d to a
     * {@code double[]}. Rotation is logged as an XYZ for ordinary use and a
     * quaternion for AdvantageScope.
     * 
     * <p>
     * {@code Rotation3d}: Logs the rotation to a {@code double[]}. Logs an XYZ and
     * a quaternion.
     * 
     * <p>
     * {@code Quaternion}: Logs the quaternion to a {@code double[]}.
     * 
     * <p>
     * {@code SwerveDriveOdometry}: Gets the {@code Pose2d} and logs it; see
     * {@code Pose2d} above. Logs in meters.
     * 
     * <p>
     * (NOTE: if neither an input nor output unit is specified for a property of an
     * object (e.g. {@code Pose2d} and location), the object in question stores that
     * as just a value and not something with internal unit conversion (for example,
     * storing a value representative of rotation as a {@code double} instead of a
     * {@code Rotation2d}). FRCLogger simply logs those values without converting
     * anything).
     * 
     * @param filePath An {@code Object} containing the "folder" that the logged
     *                 data is stored in. Use a {@code String} to specify a custom
     *                 path within the log folder (e.g. {@code DriveTrainLogs}), or
     *                 input {@code this} to automatically name the path the same
     *                 name as the object.
     * 
     * @param name     A {@code String} containing the name of the logged data (e.g.
     *                 {@code RobotPosition}). Usually the name of the function or
     *                 variable that is being logged.
     * 
     * @param data     An {@code Object} of one of the acceptable object listed
     *                 above containing the data to log.
     * 
     * @return A boolean; {@code true} if the object logged successfully or
     *         {@code false} if it didn't. Note that if this {@code FRCLogger} has
     *         logging disabled, this method will still return {@code true}.
     */
    public boolean log(Object filePath, String name, Object data) {
        if (log) {
            String path;
            if (filePath.getClass().getSimpleName().equals("String")) {
                path = (String) filePath;
            } else {
                if (filePath.getClass().getSimpleName().equals("Vision")) {
                    Vision vision = (Vision) filePath;
                    path = vision.getVisionName();
                } else {
                    path = filePath.getClass().getSimpleName();
                }
            }
            if (data.getClass().getSimpleName().equals("boolean")||data.getClass().getSimpleName().equals("Boolean")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("int")||data.getClass().getSimpleName().equals("Integer")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (int) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("double")||data.getClass().getSimpleName().equals("Double")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("String")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("boolean[]")||data.getClass().getSimpleName().equals("Boolean[]")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean[]) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("int[]")){
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data); // double[] here because int[] isn't suposrted by AK in 2023
                return true;
            }
            if (data.getClass().getSimpleName().equals("double[]")){
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("double[]")||data.getClass().getSimpleName().equals("Double[]")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data);
                return true;
            }
            if (data.getClass().getSimpleName().equals("String[]")) {
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String[]) data);
                return true;
            }
            if (data instanceof SwerveModule) {
                SwerveModule swerveModule = (SwerveModule) data;
                double[] result = new double[2];
                result[0] = swerveModule.getSteerAngle();
                result[1] = swerveModule.getDriveVelocity();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, result);
                return true;
            }
            if (data instanceof SwerveModule[]) {
                SwerveModule[] swerveModules = (SwerveModule[]) data;
                double[] result = new double[2 * swerveModules.length];
                for (int i = 0; i < swerveModules.length; i++) {
                    result[i * 2] = swerveModules[i].getSteerAngle();
                    result[i * 2 + 1] = swerveModules[i].getDriveVelocity();
                }
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, result);
                return true;
            }
            if (data instanceof Pose2d) {
                Pose2d pose = (Pose2d) data;
                double[] result = new double[3];
                result[0] = pose.getX();
                result[1] = pose.getY();
                result[2] = pose.getRotation().getDegrees();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, result);
                return true;
            }
            if (data instanceof Rotation2d) {
                Rotation2d rotation = (Rotation2d) data;
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Degrees",
                        rotation.getDegrees());
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Radians",
                        rotation.getRadians());
                return true;
            }
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
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Quaternion", result);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/XYZ", resultXYZ);
                return true;
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
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/XYZ", resultXYZ);
                return true;
            }
            if (data instanceof Quaternion) {
                Quaternion rotation = (Quaternion) data;
                double[] result = new double[4];
                result[0] = rotation.getW();
                result[1] = rotation.getX();
                result[2] = rotation.getY();
                result[3] = rotation.getZ();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, result);
                return true;
            }
            if (data instanceof SwerveDriveOdometry) {
                SwerveDriveOdometry odometry = (SwerveDriveOdometry) data;
                return log(filePath, name, odometry.getPoseMeters());
            }
            System.out.println("FRCLogger: Log failure: Attempt to log unsupported object \""
                    + data.getClass().getSimpleName() + "\"!");
            return false;
        }
        return true;
    }

    public void enableLogging() {
        log = true;
        System.out.println("FRCLogger: LOGGING ENABLED.");
    }

    public void disableLogging() {
        log = false;
        System.out.println("FRCLogger: LOGGING DISABLED.");
    }

    public void setNewLogFolder(String s) {
        logFolder = s;
    }

    /**
     * The default log folder is {@code CustomLogs}.
     */
    public void setDefaultLogFolder() {
        logFolder = "CustomLogs";
    }
}