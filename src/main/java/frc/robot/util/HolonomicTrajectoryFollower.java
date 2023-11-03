package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.List;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.TrajectoryFollowing.*;

public final class HolonomicTrajectoryFollower {
    public static SwerveDrivetrain swerveInst_m = SwerveDrivetrain.getInstance();
    //the holonomic drive controller will add an additional meter per second in the x direction for every meter of error in the x direction and will add an additional 1.2 meters per second in the y direction for every meter of error in the y direction.
    private static final PIDController xPIDController_m = new PIDController(VAL_KP_X, 0, 0);
    private static final PIDController yPIDController_m = new PIDController(VAL_KP_Y, 0, 0);
    private static final ProfiledPIDController thetaPIDController_m = new ProfiledPIDController(VAL_KP_THETA, 0, 0, new TrapezoidProfile.Constraints(VAL_MAX_VELO, VAL_MAX_ACCEL));

    private static SwerveControllerCommand swerveCmd_m;

    /**
     * Creates a new trajectory for the robot to follow with the passed in
     * waypoints
     * 
     * @param waypoints The list of waypoints you would like to pass through
     * @param reversed  If you want to drive backwards, this should be true
     * @return A trajectory, a.k.a the path for the robot to follow
     */
    public static Trajectory initializeTrajectory(List<Pose2d> waypoints, boolean reversed) {

        // Create config for trajectory 
        TrajectoryConfig config = new TrajectoryConfig(VAL_MAX_VELO,VAL_MAX_ACCEL).setKinematics(swerveInst_m.getKinematics());
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    /**
     * initialize swerve controller command by passing in trajectory to follow
     * 
     * @param traj trajectory parameter to follow
     * @return new swerve controller command to be scheduled
     */
    public static SwerveControllerCommand initializeSwerveControllerCommand(Trajectory traj) {
        swerveCmd_m = new SwerveControllerCommand(
                traj,
                swerveInst_m::getPose,
                swerveInst_m.getKinematics(),
                xPIDController_m,
                yPIDController_m,
                thetaPIDController_m,
                swerveInst_m::setDesiredModuleState,
                swerveInst_m
                );
        return swerveCmd_m;
    }
}