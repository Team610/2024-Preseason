package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final int PORT_DRIVER = 0;
    public static final int PORT_OPERATOR = 1;
    public static final double VAL_DEADBAND = 0.1;
    public static final String CAN_BUS_NAME = "Vulture";

    public static final class Swerve {
        public static final int CAN_PIDGEOTTO = 20;
        public static final boolean VAL_PIDGEOTTO_INV = false; 

        /* Drivetrain Constants */
        public static final double VAL_TRACK_WIDTH = 0.40125; 
        public static final double VAL_WHEEL_BASE = 0.40125; 
        public static final double VAL_WHEEL_CIRCUM = Units.inchesToMeters(4.0)*Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(VAL_WHEEL_BASE / 2.0, VAL_TRACK_WIDTH / 2.0),
            new Translation2d(VAL_WHEEL_BASE / 2.0, -VAL_TRACK_WIDTH / 2.0),
            new Translation2d(-VAL_WHEEL_BASE / 2.0, VAL_TRACK_WIDTH / 2.0),
            new Translation2d(-VAL_WHEEL_BASE / 2.0, -VAL_TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double VAL_DRIVE_GEAR_RATIO = (6.75 / 1.0);
        public static final double VAL_STEER_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /* Motor Inverts */
        public static final boolean VAL_STEER_MOTOR_INV = true;
        public static final boolean VAL_DRIVE_MOTOR_INV = false;

        /* Steer Encoder Invert */
        public static final boolean VAL_CANCODER_INV = false;

        /* Swerve Current Limiting */
        public static final int VAL_STEER_CONT_CUR_LIM = 25;
        public static final int VAL_STEER_PEAK_CUR_LIM = 40;
        public static final double VAL_STEER_PEAK_CUR_DUR = 0.1;
        public static final boolean VAL_STEER_ENABLE_CUR_LIM = true;

        public static final int VAL_DRIVE_CONT_CUR_LIM = 35;
        public static final int VAL_DRIVE_PEAK_CUR_LIM = 60;
        public static final double VAL_DRIVE_PEAK_CUR_DUR = 0.1;
        public static final boolean VAL_DRIVE_ENABLE_CUR_LIM = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double VAL_OPEN_LOOP_RAMP = 0.25;
        public static final double VAL_CLOSED_LOOP_RAMP = 0.0;

        /* Steer Motor PID Values */
        public static final double VAL_STEER_KP = 0.3;
        public static final double VAL_STEER_KI = 0.0;
        public static final double VAL_STEER_KD = 0.0;
        public static final double VAL_STEER_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double VAL_DRIVE_KP = 0.05; 
        public static final double VAL_DRIVE_KI = 0.0;
        public static final double VAL_DRIVE_KD = 0.0;
        public static final double VAL_DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double VAL_DRIVE_KS = (0.32 / 12); //SysID
        public static final double VAL_DRIVE_KV = (1.51 / 12);
        public static final double VAL_DRIVE_KA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double VAL_MAX_SPD = 2; 
        /** Radians per Second */
        public static final double VAL_MAX_ANG_VELO = 4; 

        /* Neutral Modes */
        public static final NeutralMode VAL_STEER_NEUTRAL = NeutralMode.Coast;
        public static final NeutralMode VAL_DRIVE_NEUTRAL = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int CAN_DRIVE_MOTOR = 1;
            public static final int CAN_STEER_MOTOR = 5;
            public static final int CAN_CANCODER = 11;
            public static final Rotation2d VAL_ANGLE_OFFSET = Rotation2d.fromDegrees(-45+(108.545+109.424+110.654)/3);
            public static final SwerveModuleConstants VAL_MODULE_CONSTANTS = 
                new SwerveModuleConstants(CAN_DRIVE_MOTOR, CAN_STEER_MOTOR, CAN_CANCODER, VAL_ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule { 
            public static final int CAN_DRIVE_MOTOR = 2;
            public static final int CAN_STEER_MOTOR = 6;
            public static final int CAN_CANCODER = 12;
            public static final Rotation2d VAL_ANGLE_OFFSET = Rotation2d.fromDegrees(45+(38.496+40.693+38.496)/3);
            public static final SwerveModuleConstants VAL_MODULE_CONSTANTS = 
                new SwerveModuleConstants(CAN_DRIVE_MOTOR, CAN_STEER_MOTOR, CAN_CANCODER, VAL_ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftModule { 
            public static final int CAN_DRIVE_MOTOR = 3;
            public static final int CAN_STEER_MOTOR = 7;
            public static final int CAN_CANCODER = 13;
            public static final Rotation2d VAL_ANGLE_OFFSET = Rotation2d.fromDegrees(180+45+(124.541+123.486+124.365)/3);
            public static final SwerveModuleConstants VAL_MODULE_CONSTANTS = 
                new SwerveModuleConstants(CAN_DRIVE_MOTOR, CAN_STEER_MOTOR, CAN_CANCODER, VAL_ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule { 
            public static final int CAN_DRIVE_MOTOR = 4;
            public static final int CAN_STEER_MOTOR = 8;
            public static final int CAN_CANCODER = 14;
            public static final Rotation2d VAL_ANGLE_OFFSET = Rotation2d.fromDegrees(180-45+(42.1+41.045+40.89)/3);
            public static final SwerveModuleConstants VAL_MODULE_CONSTANTS = 
                new SwerveModuleConstants(CAN_DRIVE_MOTOR, CAN_STEER_MOTOR, CAN_CANCODER, VAL_ANGLE_OFFSET);
        }
    }
}
