// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Travel extends CommandBase {
  
  private Swerve s_Swerve;
  


  //positions robot travels too
  private double mXPos;
  private double mYPos;

  //PID controllers to control movement in X and Y directions
  private PIDController pidX_s;
  private PIDController pidY_s;

//  private ProfiledPIDController pidX_s;
//  private ProfiledPIDController pidY_s;

  private BooleanSupplier robotCentricSup;

  /** Creates a new Travel. */
  public Travel(Swerve s_Swerve, double mXPos, double mYPos, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.mXPos = mXPos;
    this.mYPos = mYPos;


    this.pidX_s = new PIDController(0.1, 0, 0);
    this.pidY_s = new PIDController(0.1,0,0);



    /*this.pidX_s = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    this.pidY_s = new ProfiledPIDController(0.1,0,0, new TrapezoidProfile.Constraints(
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));*/




    this.robotCentricSup = robotCentricSup;

   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive(
            new Translation2d(pidX_s.calculate(s_Swerve.getPose().getX(), mXPos), pidY_s.calculate(s_Swerve.getPose().getY(), mYPos)).times(Constants.Swerve.maxSpeed),
            0, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if ((Math.abs(s_Swerve.getPose().getX() - mXPos) < 1) && (Math.abs(s_Swerve.getPose().getY() - mYPos) < 1)) {
      return true;
  } else {
      return false;
  }
  }
}
