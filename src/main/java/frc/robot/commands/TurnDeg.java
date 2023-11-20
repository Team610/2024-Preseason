package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.pigeot;


public class TurnDeg extends CommandBase {    
    private Swerve s_Swerve;    
    private double beforeYaw;
    public TurnDeg(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {

        /* Drive */
        beforeYaw = pigeot.getYaw();
        while(pigeot.getYaw()-90>beforeYaw){
            s_Swerve.drive(
                new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
                (90 /2) * 0.04, 
                !true, 
                true
            );
        }
    }
}