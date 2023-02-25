package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Waypoint {
    private double endX;
    private double endY;
    private Rotation2d endRotation;
    private double endVelocity;

    public Waypoint(double endX, double endY, Rotation2d endRotation, double endVelocity) {
        this.endX = endX;
        this.endY = endY;
        this.endRotation = endRotation;
        this.endVelocity = endVelocity;
    }

    public Pose2d getEndingPose() {
        return new Pose2d(endX, endY, endRotation);
    }

    public double getEndingVelocity() {
        return endVelocity;
    }

    // public Pose2d getDifference(Waypoint otherWaypoint) {
    //     return new Pose2d(
    //         getEndingPose().minus(otherWaypoint.getEndingPose()).getX(), 
    //         getEndingPose().minus(otherWaypoint.getEndingPose()).getY(), 
    //         endRotation.minus(otherWaypoint.getEndingPose().getRotation()));
    // }
}
