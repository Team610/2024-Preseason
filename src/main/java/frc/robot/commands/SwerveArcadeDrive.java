package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveArcadeDrive extends CommandBase {    
    private Drivetrain drivetrainInst_m;    
    private DoubleSupplier translationSup_m;
    private DoubleSupplier strafeSup_m;
    private DoubleSupplier rotationSup_m;
    private BooleanSupplier robotCentricSup_s;
    private static int counter_s;

    public SwerveArcadeDrive(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        drivetrainInst_m = Drivetrain.getInstance();
        addRequirements(drivetrainInst_m);
        translationSup_m = translationSup;
        strafeSup_m = strafeSup;
        rotationSup_m = rotationSup;
        robotCentricSup_s = robotCentricSup;
        counter_s = 0;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -1* MathUtil.applyDeadband(translationSup_m.getAsDouble(), Constants.VAL_DEADBAND);
        double strafeVal = -1*MathUtil.applyDeadband(strafeSup_m.getAsDouble(), Constants.VAL_DEADBAND);
        double rotationVal = -1 * MathUtil.applyDeadband(rotationSup_m.getAsDouble(), Constants.VAL_DEADBAND);

        /* Drive */
        if(counter_s == 0 && !(translationVal == 0 && strafeVal == 0 && rotationVal == 0)){
            drivetrainInst_m.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.VAL_MAX_SPD), 
                rotationVal * Constants.Swerve.VAL_MAX_ANG_VELO, 
                !robotCentricSup_s.getAsBoolean(), 
                true
            );
            counter_s++;
        }else if(counter_s!=0){
            drivetrainInst_m.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.VAL_MAX_SPD), 
                rotationVal * Constants.Swerve.VAL_MAX_ANG_VELO, 
                !robotCentricSup_s.getAsBoolean(), 
                true
            );
        }
    }
}