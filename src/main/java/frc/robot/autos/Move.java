package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class Move extends SequentialCommandGroup{
    public Move(Swerve s_Swerve) {
        PathPlannerTrajectory move = PathPlanner.loadPath("Group", new PathConstraints(2, 2));
        addCommands(
                new InstantCommand(() -> s_Swerve.zeroGyro()),
                new InstantCommand(() -> s_Swerve.resetSwerveModuleAngles()),
                new ParallelCommandGroup(s_Swerve.followTrajectoryCommand(move, true))
        );
    }
}
