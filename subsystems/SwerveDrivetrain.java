package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.util.Subsystem610;

public class SwerveDrivetrain extends Subsystem610{

    //! https://abhinavwastaken.medium.com/using-inverse-kinematics-to-become-a-master-swerver-1026759d81b0 (Good inverse kinematic for swerve explanation)

    private static SwerveDrivetrain driveInst_s;
    private SwerveDriveKinematics kinematics_m; //Helper class to convert chassis velocity into swerve module state 
    private SwerveDrivePoseEstimator estimator_m; //* replacement for odometry (allows for limelight use), can change back to odometry if botpose is not needed
    private SwerveModule frontLeftModule_m, frontRightModule_m, backLeftModule_m, backRightModule_m;
    private WPI_Pigeon2 pidgey_m;
    private ChassisSpeeds desiredChassisVelo_m, currentChassisVelo_m;
    private SwerveModuleState[] desiredModuleState_m;
    private SwerveModuleState currentFrontLeftModuleState_m, currentFrontRightModuleState_m, currentBackLeftModuleState_m, currentBackRightModuleState_m;

    //TODO figure out what motorOutputLimiter does in Jack in the bot's code and when they use it
    private SwerveDrivetrain(){
        //Register the subsystem to use for dashboards
        super("SwerveDrivetrain");

        desiredChassisVelo_m = new ChassisSpeeds(); currentChassisVelo_m = new ChassisSpeeds();
        
        kinematics_m = new SwerveDriveKinematics(
          // Front left
          new Translation2d(VAL_TRACK_WIDTH/2.0, VAL_WHEEL_BASE/2.0),
          // Front right
          new Translation2d(VAL_TRACK_WIDTH/2.0, -VAL_WHEEL_BASE/2.0),
          // Back left
          new Translation2d(-VAL_TRACK_WIDTH/2.0, VAL_WHEEL_BASE/2.0),
          // Back right
          new Translation2d(-VAL_TRACK_WIDTH/2.0, -VAL_WHEEL_BASE/2.0)
        );
        //TODO work on trajectory follower

        pidgey_m = new WPI_Pigeon2(CAN_PIDGEY, CAN_BUS_NAME);

        //Setup Modules
        frontLeftModule_m = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, CAN_FRONT_LEFT_DRIVE)
                .withSteerMotor(MotorType.FALCON, CAN_FRONT_LEFT_STEER)
                .withSteerEncoderPort(CAN_FRONT_LEFT_ENCODER)
                .withSteerOffset(CAN_FRONT_LEFT_OFFSET)
                .build();
        //TODO why is Offset a CANID
        frontRightModule_m = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, CAN_FRONT_RIGHT_DRIVE)
                .withSteerMotor(MotorType.FALCON, CAN_FRONT_RIGHT_STEER)
                .withSteerEncoderPort(CAN_FRONT_RIGHT_ENCODER)
                .withSteerOffset(CAN_FRONT_RIGHT_OFFSET) 
                .build();

        backLeftModule_m = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, CAN_BACK_LEFT_DRIVE)
                .withSteerMotor(MotorType.FALCON, CAN_BACK_LEFT_STEER)
                .withSteerEncoderPort(CAN_BACK_LEFT_ENCODER)
                .withSteerOffset(CAN_BACK_LEFT_OFFSET)
                .build();

        backRightModule_m = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, CAN_BACK_RIGHT_DRIVE)
                .withSteerMotor(MotorType.FALCON, CAN_BACK_RIGHT_STEER)
                .withSteerEncoderPort(CAN_BACK_RIGHT_ENCODER)
                .withSteerOffset(CAN_BACK_RIGHT_ENCODER)
                .build();

        
        estimator_m = new SwerveDrivePoseEstimator(kinematics_m, getRotation2d(), 
        new SwerveModulePosition[] {frontLeftModule_m.getPosition(), frontRightModule_m.getPosition(), backLeftModule_m.getPosition(), backRightModule_m.getPosition()},
        new Pose2d(), 
        VecBuilder.fill(0.02, 0.02, 0.01),  // estimator x, y, and rotation values
        VecBuilder.fill(0.1, 0.1, 0.01)); //vision measurement x, y, and rotation values
    }

    public static SwerveDrivetrain getInstance() {
        if (driveInst_s == null) {
            driveInst_s = new SwerveDrivetrain();
        }
        return driveInst_s;
    }

    public Pose2d getPose(){return estimator_m.getEstimatedPosition();}
    public SwerveDriveKinematics getKinematics(){return kinematics_m;}
    public ChassisSpeeds getCurrentVelocity(){return currentChassisVelo_m;}

    public void drive(ChassisSpeeds chassisSpeeds){this.desiredChassisVelo_m = chassisSpeeds;}
    public void stop(){
        drive(new ChassisSpeeds(0, 0, 0));
    }
    public void setPose(Pose2d newPose){estimator_m.resetPosition(getRotation2d(),new SwerveModulePosition[] {frontLeftModule_m.getPosition(), frontRightModule_m.getPosition(), backLeftModule_m.getPosition(), backRightModule_m.getPosition()}, newPose);}
    public double getHeading(){return (pidgey_m.getYaw())%360;};
    public Rotation2d getRotation2d(){return Rotation2d.fromDegrees(getHeading());}

    public void setDesiredModuleState(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState_m, VAL_MAX_VELO);
        frontLeftModule_m.set(desiredModuleState_m[0].speedMetersPerSecond / VAL_MAX_VELO * VAL_MAX_VOLTS, desiredModuleState_m[0].angle.getRadians());
        frontRightModule_m.set(desiredModuleState_m[1].speedMetersPerSecond /VAL_MAX_VELO * VAL_MAX_VOLTS, desiredModuleState_m[1].angle.getRadians());
        backLeftModule_m.set(desiredModuleState_m[2].speedMetersPerSecond / VAL_MAX_VELO * VAL_MAX_VOLTS, desiredModuleState_m[2].angle.getRadians());
        backRightModule_m.set(desiredModuleState_m[3].speedMetersPerSecond / VAL_MAX_VELO * VAL_MAX_VOLTS, desiredModuleState_m[3].angle.getRadians());
    }
    
    public void periodic() {
        currentFrontLeftModuleState_m = new SwerveModuleState(frontLeftModule_m.getDriveVelocity(),new Rotation2d(frontLeftModule_m.getSteerAngle()));
        currentFrontRightModuleState_m = new SwerveModuleState(frontRightModule_m.getDriveVelocity(),new Rotation2d(frontRightModule_m.getSteerAngle()));
        currentBackLeftModuleState_m = new SwerveModuleState(backLeftModule_m.getDriveVelocity(),new Rotation2d(backLeftModule_m.getSteerAngle()));
        currentBackRightModuleState_m = new SwerveModuleState(backRightModule_m.getDriveVelocity(),new Rotation2d(backRightModule_m.getSteerAngle()));

        currentChassisVelo_m = kinematics_m.toChassisSpeeds(currentFrontLeftModuleState_m, currentFrontRightModuleState_m, currentBackLeftModuleState_m, currentBackRightModuleState_m);
        
        estimator_m.update(getRotation2d(), new SwerveModulePosition[] {frontLeftModule_m.getPosition(), frontRightModule_m.getPosition(), backLeftModule_m.getPosition(), backRightModule_m.getPosition()});

        desiredModuleState_m = kinematics_m.toSwerveModuleStates(desiredChassisVelo_m);
        setDesiredModuleState(desiredModuleState_m);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    @Override
    public void addToDriveTab(ShuffleboardTab tab) {
    }
}