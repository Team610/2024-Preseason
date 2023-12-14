// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class T_Vision_Spin extends CommandBase {
    PIDController pidAng_s = new PIDController(0.035, 0, 0);
    Swerve swerveInst_s;
    Vision visionInst_s;
    private boolean actInPos = false;
    int isInPosCnt = 0;
    public T_Vision_Spin(Swerve swerveInst_s, Vision visionInst_s) {
        this.swerveInst_s = swerveInst_s;
        this.visionInst_s = visionInst_s;
        addRequirements(swerveInst_s, visionInst_s);
    }

    @Override
    public void initialize() {
        //! this should be fine
        pidAng_s.setSetpoint(0);
        pidAng_s.setTolerance(10);
        visionInst_s.setPipeline(0); 
        isInPosCnt = 0;
        actInPos = false;
    }

    @Override
    public void execute() {
        double degrees = swerveInst_s.getYaw().getDegrees() % 360;
            double rSpeed;
            if (Math.abs(swerveInst_s.getYaw().getDegrees()) < 10) {
                rSpeed = 0;
                swerveInst_s.stop();
                actInPos = true;
            }else{
                rSpeed = pidAng_s.calculate(degrees);
            }
            //double rSpeed = 0;
            double xSpeed = 0;
            double ySpeed = 0;
            Translation2d translation = new Translation2d(ySpeed, xSpeed);
            SmartDashboard.putNumber("AngleError", pidAng_s.getPositionError());
            SmartDashboard.putNumber("rSpeed",rSpeed);
            SmartDashboard.putNumber("Degree", degrees);
            swerveInst_s.drive(translation, rSpeed, true, true);
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