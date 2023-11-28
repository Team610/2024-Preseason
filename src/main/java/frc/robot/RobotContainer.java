// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.T_Vision_Light;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class RobotContainer {
  public static WPI_Pigeon2 pidgey_s;
  public static Vision visionInst_s;
  public RobotContainer() {
    System.out.println("yo");
    visionInst_s = Vision.getInstance();
    visionInst_s.setDefaultCommand(new T_Vision_Light());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
