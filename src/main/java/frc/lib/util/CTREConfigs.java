package frc.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import static frc.robot.Constants.Swerve.*;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig_s;
    public static TalonFXConfiguration swerveDriveFXConfig_s;
    public static CANCoderConfiguration swerveCanCoderConfig_s;
    private SupplyCurrentLimitConfiguration driveSupplyLimit_m;

    public CTREConfigs(){
        swerveAngleFXConfig_s = new TalonFXConfiguration();
        swerveDriveFXConfig_s = new TalonFXConfiguration();
        swerveCanCoderConfig_s = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            VAL_STEER_ENABLE_CUR_LIM, 
            VAL_STEER_CONT_CUR_LIM, 
            VAL_STEER_PEAK_CUR_LIM, 
            VAL_STEER_PEAK_CUR_DUR);

        swerveAngleFXConfig_s.slot0.kP = VAL_STEER_KP;
        swerveAngleFXConfig_s.slot0.kI = VAL_STEER_KI;
        swerveAngleFXConfig_s.slot0.kD = VAL_STEER_KD;
        swerveAngleFXConfig_s.slot0.kF = VAL_STEER_KF;
        swerveAngleFXConfig_s.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        driveSupplyLimit_m = new SupplyCurrentLimitConfiguration(
            VAL_DRIVE_ENABLE_CUR_LIM, 
            VAL_DRIVE_CONT_CUR_LIM, 
            VAL_DRIVE_PEAK_CUR_LIM, 
            VAL_DRIVE_PEAK_CUR_DUR);

        swerveDriveFXConfig_s.slot0.kP = VAL_DRIVE_KP;
        swerveDriveFXConfig_s.slot0.kI = VAL_DRIVE_KI;
        swerveDriveFXConfig_s.slot0.kD = VAL_DRIVE_KD;
        swerveDriveFXConfig_s.slot0.kF = VAL_DRIVE_KF;        
        swerveDriveFXConfig_s.supplyCurrLimit = driveSupplyLimit_m;
        swerveDriveFXConfig_s.openloopRamp = VAL_OPEN_LOOP_RAMP;
        swerveDriveFXConfig_s.closedloopRamp = VAL_CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig_s.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig_s.sensorDirection = VAL_CANCODER_INV;
        swerveCanCoderConfig_s.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig_s.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}