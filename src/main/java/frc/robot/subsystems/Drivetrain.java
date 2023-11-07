package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.Subsystem610;

public class Drivetrain extends Subsystem610 {
    private static Drivetrain drivetrainInst_s;
    private SwerveDriveOdometry swerveOdometry_m;
    private SwerveModule[] swerveModules_m;
    private SwerveModuleState[] currentModuleStates_m;
    private SwerveModuleState[] desiredModuleStates_m;
    private SwerveModulePosition[] modulePositions_m;
    private Pigeon2 pigeotto_m;

    public Drivetrain() {
        super("Drivetrain");
        pigeotto_m = new Pigeon2(CAN_PIDGEOTTO, CAN_BUS_NAME);
        pigeotto_m.configFactoryDefault();
        zeroGyro();

        swerveModules_m = new SwerveModule[] {
            new SwerveModule(0, FrontLeftModule.VAL_MODULE_CONSTANTS),
            new SwerveModule(1, FrontRightModule.VAL_MODULE_CONSTANTS),
            new SwerveModule(2, BackLeftModule.VAL_MODULE_CONSTANTS),
            new SwerveModule(3, BackRightModule.VAL_MODULE_CONSTANTS)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry_m = new SwerveDriveOdometry(SWERVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public static Drivetrain getInstance() {
        if (drivetrainInst_s == null) {
            drivetrainInst_s = new Drivetrain();
        }
        return drivetrainInst_s;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        desiredModuleStates_m =
            SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates_m, VAL_MAX_SPD);

        for(SwerveModule mod : swerveModules_m){
            mod.setDesiredState(desiredModuleStates_m[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, VAL_MAX_SPD);
        
        for(SwerveModule mod : swerveModules_m){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry_m.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry_m.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        currentModuleStates_m = new SwerveModuleState[4];
        for(SwerveModule mod : swerveModules_m){
            currentModuleStates_m[mod.moduleNumber] = mod.getState();
        }
        return currentModuleStates_m;
    }

    public SwerveModulePosition[] getModulePositions(){
        modulePositions_m = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules_m){
            modulePositions_m[mod.moduleNumber] = mod.getPosition();
        }
        return modulePositions_m;
    }

    public void zeroGyro(){
        pigeotto_m.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (VAL_PIDGEOTTO_INV) ? Rotation2d.fromDegrees(-pigeotto_m.getYaw()) : Rotation2d.fromDegrees(pigeotto_m.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveModules_m){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry_m.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : swerveModules_m){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
    }

    @Override
    public void addToDriveTab(ShuffleboardTab tab) {
        // TODO Auto-generated method stub
    }
}