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
    PIDController pidX_s = new PIDController(0.04, 0, 0);
    PIDController pidY_s = new PIDController(0.08, 0, 0);
    PIDController pidAng_s = new PIDController(0.08, 0, 0);
    Swerve swerveInst_s;
    Vision visionInst_s;
    private boolean actInPos;
    int isInPosCnt = 0;
    public T_Vision_Drive(Swerve swerveInst_s, Vision visionInst_s) {
        this.swerveInst_s = swerveInst_s;
        this.visionInst_s = visionInst_s;
        addRequirements(swerveInst_s, visionInst_s);
    }

    @Override
    public void initialize() {
        pidX_s.setSetpoint(0); //! this should be fine
        pidAng_s.setSetpoint(180);
        pidY_s.setSetpoint(-10); 
        pidX_s.setTolerance(1);
        pidY_s.setTolerance(1);
        //pidAng_s.setTolerance(30);
        visionInst_s.setPipeline(0); 
        isInPosCnt = 0;
        actInPos = false;
    }

    @Override
    public void execute() {
        if(!visionInst_s.isReady()) {actInPos = true; System.out.println("sad david sounds");}
        if (visionInst_s.getTv() == 1) {
            double degrees = swerveInst_s.getYaw().getDegrees();
            if (degrees < 0) degrees += 360;
            //double rSpeed = -1*pidAng_s.calculate(degrees);
            double rSpeed = 0;
            double xSpeed = -1*pidX_s.calculate(visionInst_s.calcTx());
            double ySpeed = -1*pidY_s.calculate(visionInst_s.calcTy());
            // double xSpeed = 0;
            // double ySpeed = 0;
            Translation2d translation = new Translation2d(ySpeed, xSpeed);
            SmartDashboard.putNumber("AngleError", pidAng_s.getPositionError());
            SmartDashboard.putNumber("Y Error", pidY_s.getPositionError());
            SmartDashboard.putNumber("X Error", pidX_s.getPositionError());
            SmartDashboard.putNumber("xSpeed",xSpeed);
            SmartDashboard.putNumber("ySpeed",ySpeed);
            SmartDashboard.putNumber("rSpeed",rSpeed);
            
            swerveInst_s.drive(translation, rSpeed, false, true);
            if (Math.abs(visionInst_s.calcTy() + 10) < 0.5) isInPosCnt++; //TODO tune???
            else isInPosCnt = 0;

            if (isInPosCnt > 5) actInPos = true;
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