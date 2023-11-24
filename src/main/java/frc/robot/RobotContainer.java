// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;

public class RobotContainer {
  public static CommandXboxController driver_s;
  public static CommandXboxController operator_s;
  public static WPI_Pigeon2 pidgey_s;
  public static Vision visionInst_s;
  public RobotContainer() {
    configureBindings();
    driver_s = new CommandXboxController(PORT_DRIVER);
    operator_s = new CommandXboxController(PORT_OPERATOR);
    pidgey_s = new WPI_Pigeon2(CAN_PIDGEY, CAN_BUS_NAME);
    visionInst_s = Vision.getInstance();
  }
  private void configureBindings() {}
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
