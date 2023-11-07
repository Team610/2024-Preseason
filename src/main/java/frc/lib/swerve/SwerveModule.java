package frc.lib.swerve;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREConfigs;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber_m;
    private Rotation2d angleOffset_m;
    private Rotation2d lastAngle_m;

    private TalonFX steerMotor_m;
    private TalonFX driveMotor_m;
    private CANCoder CANCoder_m;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(VAL_DRIVE_KS, VAL_DRIVE_KV, VAL_DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        moduleNumber_m = moduleNumber;
        angleOffset_m = moduleConstants.VAL_ANGLE_OFFSET;
        
        /* Angle Encoder Config */
        CANCoder_m = new CANCoder(moduleConstants.CAN_CANCODER, CAN_BUS_NAME);
        configCANCoder_m();

        /* Angle Motor Config */
        steerMotor_m = new TalonFX(moduleConstants.CAN_STEER_MOTOR, CAN_BUS_NAME);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor_m = new TalonFX(moduleConstants.CAN_DRIVE_MOTOR, CAN_BUS_NAME);
        configDriveMotor();

        lastAngle_m = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / VAL_MAX_SPD;
            driveMotor_m.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, VAL_WHEEL_CIRCUM, VAL_DRIVE_GEAR_RATIO);
            driveMotor_m.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (VAL_MAX_SPD * 0.01)) ? lastAngle_m : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        steerMotor_m.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), VAL_STEER_GEAR_RATIO));
        lastAngle_m = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor_m.getSelectedSensorPosition(), VAL_STEER_GEAR_RATIO));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(CANCoder_m.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset_m.getDegrees(), VAL_STEER_GEAR_RATIO);
        steerMotor_m.setSelectedSensorPosition(absolutePosition);
    }

    private void configCANCoder_m(){        
        CANCoder_m.configFactoryDefault();
        CANCoder_m.configAllSettings(CTREConfigs.swerveCanCoderConfig_s);
    }

    private void configAngleMotor(){
        steerMotor_m.configFactoryDefault();
        steerMotor_m.configAllSettings(CTREConfigs.swerveAngleFXConfig_s);
        steerMotor_m.setInverted(VAL_STEER_MOTOR_INV);
        steerMotor_m.setNeutralMode(VAL_STEER_NEUTRAL);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor_m.configFactoryDefault();
        driveMotor_m.configAllSettings(CTREConfigs.swerveDriveFXConfig_s);
        driveMotor_m.setInverted(VAL_DRIVE_MOTOR_INV);
        driveMotor_m.setNeutralMode(VAL_DRIVE_NEUTRAL);
        driveMotor_m.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor_m.getSelectedSensorVelocity(), VAL_WHEEL_CIRCUM, VAL_DRIVE_GEAR_RATIO), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor_m.getSelectedSensorPosition(), VAL_WHEEL_CIRCUM, VAL_DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }
}