package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Drivetrain.*;

public class T_SwerveDrivetrain_FieldOriented extends CommandBase {
    private SwerveDrivetrain drivetrainInst_m;

    public T_SwerveDrivetrain_FieldOriented() {
        drivetrainInst_m = SwerveDrivetrain.getInstance();
        addRequirements(drivetrainInst_m);
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;
        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private double getForwardInput() {
        return -deadband(RobotContainer.driver_s.getLeftY(), 0.1) * VAL_MAX_VELO;
    }

    private double getStrafeInput() {
        return -deadband(RobotContainer.driver_s.getLeftX(), 0.1) * VAL_MAX_VELO;
    }

    private double getRotationInput() {
        return -deadband(RobotContainer.driver_s.getRightX(), 0.1) * VAL_MAX_VELO;
    }

    @Override
    public void execute() {
        double gyroAngle = drivetrainInst_m.getHeading();

        double forwardInput = getForwardInput();
        double strafeInput = getStrafeInput();
        double rotationInput = getRotationInput();

        double fieldForward = forwardInput * Math.cos(Math.toRadians(gyroAngle)) - strafeInput * Math.sin(Math.toRadians(gyroAngle));
        double fieldStrafe = forwardInput * Math.sin(Math.toRadians(gyroAngle)) + strafeInput * Math.cos(Math.toRadians(gyroAngle));

        drivetrainInst_m.drive(new ChassisSpeeds(fieldForward, fieldStrafe, rotationInput));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainInst_m.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
