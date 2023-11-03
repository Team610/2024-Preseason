package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Drivetrain.*;


/**
 * Default teleop drive mode
 */
public class T_SwerveDrivetrain_RobotOriented extends CommandBase {
    private SwerveDrivetrain drivetrainInst_m;

    public T_SwerveDrivetrain_RobotOriented() {
        drivetrainInst_m = SwerveDrivetrain.getInstance();
        addRequirements(drivetrainInst_m);
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;
        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }
  
    private static double square(double value) {
        return Math.copySign(value * value, value);
    }
    //TODO test if negative sign is needed
    
    private double getForwardInput() {
        return -square(deadband(RobotContainer.driver_s.getLeftY(), 0.1)) * VAL_MAX_VELO;
    }
  
    private double getStrafeInput() {
        return -square(deadband(RobotContainer.driver_s.getLeftX(), 0.1)) * VAL_MAX_VELO;
    }
  
    private double getRotationInput() {
        return -square(deadband(RobotContainer.driver_s.getRightX(), 0.1)) * VAL_MAX_VELO;
    }
  

    @Override
    public void execute() {
        drivetrainInst_m.drive(ChassisSpeeds.fromFieldRelativeSpeeds(getForwardInput(), getStrafeInput(), getRotationInput(), drivetrainInst_m.getRotation2d()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainInst_m.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}