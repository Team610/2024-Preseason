// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class T_Vision_Drive extends CommandBase {
    Swerve swerveInst_s;
    Vision visionInst_s;
    double timeStart;
    double currentTime;
    public T_Vision_Drive(Swerve swerveInst_s, Vision visionInst_s) {
        this.swerveInst_s = swerveInst_s;
        this.visionInst_s = visionInst_s;
        addRequirements(swerveInst_s, visionInst_s);
    }

    @Override
    public void initialize() {
        timeStart = Timer.getFPGATimestamp();
        visionInst_s.setInPos();
        visionInst_s.resetPID();
        visionInst_s.turnOn();
        System.out.println("on");
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        visionInst_s.drive(timeStart, currentTime);
    }

    @Override
    public void end(boolean interrupted) {
        // RobotContainer.visionInst_s.setCamMode(1);
        // RobotContainer.visionInst_s.setLedMode(1);
        visionInst_s.turnOff();
        swerveInst_s.stop();
    }

    @Override
    public boolean isFinished() {
        return visionInst_s.isInPos();
    }
}