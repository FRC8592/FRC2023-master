package frc.robot.autonomous.autons.right;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.PipelineCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.PipelineCommand.Pipeline;
import static frc.robot.autonomous.AutonomousPositions.*;

public class RightConeGrabCubeBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(3.0, 1);

    private SwerveTrajectory G_TO_Icc = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(false),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2),
        INTERMEDIARY_CABLE_COVER.translate(-1.0, -0.1),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1)
    );

    private SwerveTrajectory Icc_TO_GP4 = generate(
        config
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(-0.0, -0.1),
        GAME_PIECE_4.translate(0.25, 0.05)
    );

    private SwerveTrajectory GP4_TO_BM = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_4.translate(0.25, 0.05),
        BALANCE_MIDDLE.getPose()
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // PRIME 4-bar
            new JointCommand( // Lift elevator HIGH and score
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Lift elevator PRIME
            new PipelineCommand(vision, Pipeline.CUBE), // Change Pipeline CUBE
            new JointCommand( // STOW 4-bar and INTAKE while moving out community
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_Icc.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5)),
                new IntakeCommand(intake, 1.0, true)
            ),
            new JointCommand( // TRACK and INTAKE cube
                new FollowerCommand(drive, vision, Icc_TO_GP4.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.0).addVision()),
                new IntakeCommand(intake, true)
            ),
            new FollowerCommand(drive, GP4_TO_BM), // Move to Charging Station
            new AutobalanceCommand(drive) // BALANCE
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}