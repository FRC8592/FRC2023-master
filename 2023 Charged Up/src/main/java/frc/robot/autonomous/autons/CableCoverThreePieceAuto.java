package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

public class CableCoverThreePieceAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(5, 3);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(3, 2);

    @Override
    public void initialize() {
        SwerveTrajectory H_TO_Icc = generateTrajectoryFromPoints(
            slowConfig
                .setReversed(false)
                .setStartVelocity(0.0)
                .setEndVelocity(slowConfig.getMaxVelocity())
                .setKinematics(drive.getKinematics())
            ,
            GRID_H.getPose(),
            INTERMEDIARY_CABLE_COVER.translate(0, 0.5)
        );

        SwerveTrajectory Icc_TO_GP4 = generateTrajectoryFromPoints(
            fastConfig
                .setReversed(false)
                .setStartVelocity(slowConfig.getMaxVelocity())
                .setEndVelocity(0.0)
                .setKinematics(drive.getKinematics())
            ,
            INTERMEDIARY_CABLE_COVER.translate(0, 0.5),
            GAME_PIECE_4.getPose()
        );

        SwerveTrajectory GP4_TO_Icc = generateTrajectoryFromPoints(
            fastConfig
                .setReversed(true)
                .setStartVelocity(0.0)
                .setEndVelocity(slowConfig.getMaxVelocity())
                .setKinematics(drive.getKinematics())
            ,
            GAME_PIECE_4.getPose(),
            INTERMEDIARY_CABLE_COVER.translate(-0.5, 0)
        );

        SwerveTrajectory Icc_TO_I = generateTrajectoryFromPoints(
            slowConfig
                .setReversed(true)
                .setStartVelocity(slowConfig.getMaxVelocity())
                .setEndVelocity(0.0)
                .setKinematics(drive.getKinematics())
            ,
            INTERMEDIARY_CABLE_COVER.translate(-0.5, 0),
            GRID_I.getPose()
        );

        SwerveTrajectory I_TO_Icc = generateTrajectoryFromPoints(
            slowConfig
                .setReversed(false)
                .setStartVelocity(0.0)
                .setEndVelocity(slowConfig.getMaxVelocity())
                .setKinematics(drive.getKinematics())
            ,
            GRID_I.getPose(),
            INTERMEDIARY_CABLE_COVER.getPose()
        );

        SwerveTrajectory Icc_TO_GP3 = generateTrajectoryFromPoints(
            fastConfig
                .setReversed(false)
                .setStartVelocity(slowConfig.getMaxVelocity())
                .setEndVelocity(0.0)
                .setKinematics(drive.getKinematics())
            ,
            INTERMEDIARY_CABLE_COVER.getPose(),
            GAME_PIECE_3.rotate(Rotation2d.fromDegrees(45))
        );

        SwerveTrajectory GP3_TO_Icc = generateTrajectoryFromPoints(
            fastConfig
                .setReversed(true)
                .setStartVelocity(0.0)
                .setEndVelocity(slowConfig.getMaxVelocity())
                .setKinematics(drive.getKinematics())
            ,
            GAME_PIECE_3.getPose(),
            INTERMEDIARY_CABLE_COVER.getPose()
        );

        SwerveTrajectory Icc_TO_G = generateTrajectoryFromPoints(
            fastConfig
                .setReversed(true)
                .setStartVelocity(slowConfig.getMaxVelocity())
                .setEndVelocity(0.0)
                .setKinematics(drive.getKinematics())
            ,
            INTERMEDIARY_CABLE_COVER.getPose(),
            COMMUNITY_CABLE_COVER.getPose(),
            GRID_G.getPose()
        );

        queue = new CommandQueue(
            new ScoreCommand(intake), // Score preload game piece
            new JointCommand( // Lower lift; go to and intake second game piece
                new FollowerCommand(drive, H_TO_Icc),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_GP4.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake)
            ),
            new FollowerCommand(drive, GP4_TO_Icc), // Go to scoring grid and prime elevator
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_I)
                // new LiftCommand(lift, Heights.PRIME)
            ),
            new LiftCommand(elevator, Heights.HIGH), // Lift to high height and score second piece
            new ScoreCommand(intake),
            new JointCommand( // Lower lift; go to and intake third game piece
                new FollowerCommand(drive, I_TO_Icc),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_GP3.addRotation(Rotation2d.fromDegrees(-135))),
                new IntakeCommand(intake)
            ),
            new FollowerCommand(drive, GP3_TO_Icc), // Go to scoring grid and prime elevator
            new JointCommand(
                new FollowerCommand(drive, Icc_TO_G)
                // new LiftCommand(lift, Heights.PRIME)
            ),
            new LiftCommand(elevator, Heights.HIGH), // Lift to high height and score third piece
            new ScoreCommand(intake)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}