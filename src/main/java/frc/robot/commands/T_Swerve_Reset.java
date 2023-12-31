package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class T_Swerve_Reset extends CommandBase {
    Swerve swerveInst_s;
    boolean finished;

    public T_Swerve_Reset(Swerve swerveInst_s) {
        this.swerveInst_s = swerveInst_s;
        finished = false;
        addRequirements(swerveInst_s);
    }

    @Override
    public void initialize() {
        swerveInst_s.resetSwerveModuleAngles();
        finished = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        swerveInst_s.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
