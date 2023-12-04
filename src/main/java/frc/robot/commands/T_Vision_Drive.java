// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class T_Vision_Drive extends CommandBase {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    //TODO untuned values make me sad
    private final ProfiledPIDController pidX_s = new ProfiledPIDController(0.1, 0.001, 0, X_CONSTRAINTS);
    PIDController pidY_s = new PIDController(0.2, 0.01, 0);
    PIDController pidAng_s = new PIDController(0.15, 0.0005, 0);
    Swerve swerveInst_s;
    Vision visionInst_s;
    private boolean actInPos;
    int isInPosCnt = 0;
    public T_Vision_Drive(Swerve swerveInst_s, Vision visionInst_s) {
        this.swerveInst_s = swerveInst_s;
        addRequirements(swerveInst_s, visionInst_s);
    }

    @Override
    public void initialize() {
        pidX_s.setGoal(0); //! this should be fine
        pidAng_s.setSetpoint(180);
        pidY_s.setSetpoint(0); //TODO Hello darkness my old friend
        pidX_s.setTolerance(1);
        pidY_s.setTolerance(0.5);
        visionInst_s.setPipeline(0); //TODO make sure pipeline is right
        isInPosCnt = 0;
        actInPos = false;
    }

    @Override
    public void execute() {
        if (visionInst_s.getTv() == 1) {
            double degrees =swerveInst_s.getYaw().getDegrees();
            if (degrees < 0) degrees += 360;
            double rSpeed = pidAng_s.calculate(degrees);
            double xSpeed = pidX_s.calculate(visionInst_s.calcTx());
            double ySpeed = pidY_s.calculate(visionInst_s.calcTy());

            Translation2d translation = new Translation2d(ySpeed, xSpeed);
            SmartDashboard.putNumber("xSpeed",xSpeed);
            SmartDashboard.putNumber("ySpeed",ySpeed);
            SmartDashboard.putNumber("rSpeed",rSpeed);
            
            swerveInst_s.drive(translation, rSpeed, false, true);
            if (Math.abs(visionInst_s.calcTy() - 0) < 0.5) isInPosCnt++; //TODO tune???
            else isInPosCnt = 0;

            if (isInPosCnt > 20) actInPos = true;
        }
        else {
            swerveInst_s.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveInst_s.stop();
    }

    @Override
    public boolean isFinished() {
        return actInPos;
    }
}