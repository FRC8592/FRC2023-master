package frc.robot.autonomous.autons.cablecover;

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
import frc.robot.commands.ScoreCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class CableCoverPreloadGrabPieceBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 2);

    private SwerveTrajectory C_TO_Ilz = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
        GAME_PIECE_1.translate(-0.5, 0)
    );

    private SwerveTrajectory GP1_TO_BM = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-0.5, 0),
        BALANCE_MIDDLE.translate(0.25, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up 4-bar
            new JointCommand( // Lift to HIGH and score pre-load
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Retract to STOWED
            new JointCommand( // Tilt down 4-bar and drive to edge of community
                new FollowerCommand(drive, C_TO_Ilz),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new JointCommand( // Drive to cube while turning and intaking and continue stowing if not stowed yet
                new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake, true),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new FollowerCommand(drive, GP1_TO_BM), // Drive to charging station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
