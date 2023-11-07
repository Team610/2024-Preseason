package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int CAN_DRIVE_MOTOR;
    public final int CAN_STEER_MOTOR;
    public final int CAN_CANCODER;
    public final Rotation2d VAL_ANGLE_OFFSET;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param CAN_DRIVE_MOTOR
     * @param CAN_STEER_MOTOR
     * @param CAN_CANCODER
     * @param VAL_ANGLE_OFFSET
     */
    public SwerveModuleConstants(int driveMotorID, int steerMotorID, int CANCoderID, Rotation2d angleOffset) {
        CAN_DRIVE_MOTOR = driveMotorID;
        CAN_STEER_MOTOR = steerMotorID;
        CAN_CANCODER = CANCoderID;
        VAL_ANGLE_OFFSET = angleOffset;
    }
}
