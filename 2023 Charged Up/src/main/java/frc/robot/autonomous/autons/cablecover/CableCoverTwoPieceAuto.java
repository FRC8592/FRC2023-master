package frc.robot.autonomous.autons.cablecover;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class CableCoverTwoPieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory G_TO_Icc = generateTrajectoryFromPoints(
        config
            .setReversed(false)
            .setStartVelocity(0)
            .setEndVelocity(0),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, 0.5),
        INTERMEDIARY_CABLE_COVER.translate(-0.5, 0.5)
    );

    private SwerveTrajectory Icc_TO_GP4 = generateTrajectoryFromPoints(
        config
            .setReversed(false)
            .setStartVelocity(0)
            .setEndVelocity(0),
        INTERMEDIARY_CABLE_COVER.translate(-0.5, 0.5),
        INTERMEDIARY_CABLE_COVER.translate(0.25, 0.5),
        GAME_PIECE_4.getPose()
    );

    private SwerveTrajectory GP4_TO_Icc = generateTrajectoryFromPoints(
        config
            .setReversed(true)
            .setStartVelocity(0)
            .setEndVelocity(0),
        GAME_PIECE_4.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(0.25, 0.5),
        INTERMEDIARY_CABLE_COVER.translate(-0.5, 0.5)
    );

    private SwerveTrajectory Icc_TO_I = generateTrajectoryFromPoints(
        config
            .setReversed(true)
            .setStartVelocity(0)
            .setEndVelocity(0),
        INTERMEDIARY_CABLE_COVER.translate(-0.5, 0.5),
        GRID_I.translate(0.25, 0.0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high and score pre-load
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Retract the elevator
            new JointCommand( // Retract the 4-bar drive out community 
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_Icc)
            ),
            new JointCommand( // Drive to second piece and intake
                new FollowerCommand(drive, Icc_TO_GP4.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake, 0.25, true)
            ),
            new JointCommand( // Continue intaking
                new IntakeCommand(intake),
                new DelayCommand(0.5)
            ),
            new FollowerCommand(drive, GP4_TO_Icc.addRotation(Rotation2d.fromDegrees(180))), // Drive back to community
            new JointCommand( // Turn and drive to grid while tilting 4-bar
              new FollowerCommand(drive, Icc_TO_I),
              new LiftCommand(elevator, Heights.PRIME) 
            ),
            new JointCommand( // Lift to high and score second piece
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED) // Stow elevator
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
