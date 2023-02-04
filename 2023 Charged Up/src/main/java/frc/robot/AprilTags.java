package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTags extends Vision{
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry botpose;
    NetworkTableEntry ID;
    public static FRCLogger logger;
    NetworkTableEntry camerapose;

    //read values periodically
    double tagID;
    double x;
    double y;
    double area;
    double[] megapose;
    double[] limelightpose;
    
    
    
    public AprilTags(String limelightName, double lockError, double closeError,
                    double cameraHeight, double cameraAngle, double targetHeight,
                    double rotationKP, double rotationKI, double rotationKD){
        

        super(limelightName, lockError , closeError, cameraHeight, cameraAngle, targetHeight, rotationKP, rotationKI, rotationKD, logger);
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        
        this.botpose = table.getEntry("botpose");
        this.camerapose = table.getEntry("camerapose_targetspace");
        this.ID = table.getEntry("tid");
        
        limelightpose = camerapose.getDoubleArray(new double[0]);
        tagID  = ID.getDouble(0);
    }



    public ObservationNode getObservation(){
        Pose3d pose3d = new Pose3d();
        double[] pose;
        if(botpose.getDoubleArray(new double[0]) == null){
            return new ObservationNode(null, false);
        }else{
            pose = this.botpose.getDoubleArray(new double[0]);
        }

        boolean valid = pose.length != 0;

        if(valid){
            pose3d = new Pose3d(pose[0], pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
        }

        return new ObservationNode(pose3d, valid);
    }

    public class ObservationNode{
        public Pose3d pose;
        public boolean valid;
        
        public ObservationNode(Pose3d pose, boolean valid){
            this.pose = pose;
            this.valid = valid;
        
        }

        public String toString(){
            String poseString;
            poseString = "";
            if(this.valid){
                poseString = "x: " + pose.getX() + "y: " + pose.getY() + "z: "
                 + pose.getZ() + "pitch: " + pose.getRotation().getX() + "roll: "  
                 + pose.getRotation().getY() + "yaw: " + pose.getRotation().getZ();

            }else{

                poseString = "not Valid";

            }

            return poseString;
        }
    }
}
