package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Spin extends CommandBase{

    private Swerve s_Swerve;
    private double mAngle;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private double rotation;
    
    public Spin(Swerve s_Swerve, double angle, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.mAngle = angle;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
       
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {


        double translationVal = -1* MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = -1*MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        //Only using proportional term of PID, IDK how to use the PID controller class
        double anglePoisiton = s_Swerve.getYaw().getDegrees();
        double error = mAngle - anglePoisiton;
        rotation = -0.01 * error;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotation /*rotationVal*/ * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(s_Swerve.getYaw().getDegrees() - mAngle) < 5) {
            return true;
        } else {
            return false;
        }
    }



}
