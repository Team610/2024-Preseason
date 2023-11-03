package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class A_testTrajectory extends SequentialCommandGroup {
        private Trajectory test_m;

        /**
         * Add all the commands you would like to happen in auto to this, in order of
         * occurence
         */
        public A_testTrajectory() {
                String testFileLoc = "paths/output/test.wpilib.json";
                Path testPath = Filesystem.getDeployDirectory().toPath().resolve(testFileLoc);
                addRequirements(RobotContainer.drivetrainInst_s);

                test_m = null;
                try {
                        test_m = TrajectoryUtil.fromPathweaverJson(testPath);
                } catch (IOException e) {
                        e.printStackTrace();
                }

                addCommands(
                        new A_SwervePath(test_m)
                );
        }
}