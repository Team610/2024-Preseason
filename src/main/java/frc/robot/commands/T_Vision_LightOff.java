package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class T_Vision_LightOff extends CommandBase{

    public T_Vision_LightOff() {
        addRequirements(RobotContainer.visionInst_s);

    }

    /**
     * Turns limelight on
     */
    @Override
    public void initialize() {
        //RobotContainer.visionInst_s.setCamMode(1);
        RobotContainer.visionInst_s.setLedMode(1);
    }

    /**
     * Aims limelight
     */
    @Override
    public void execute() {
    }

    /**
     * Turns limelight off
     */
    @Override
    public void end(boolean interrupted) {
    }    

    @Override
    public boolean isFinished() {
        // return RobotContainer.visionInst_s.checkAim();
        return false;
    }
}