package frc.robot;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

public final class Constants {
    public static final String CAN_BUS_NAME = "Vulture";
    public static final int PORT_DRIVER = 0;
    public static final int PORT_OPERATOR = 1;
    public static final double VAL_DEADBAND = 0.03;
    public static class Drivetrain{
        //TODO change CANID values
        public static final int CAN_PIDGEY = 1;
        //? Come up for name for motors like batman and robin but for drive and steer???
        //TODO change offset (Instructions: https://github.com/SwerveDriveSpecialties/swerve-template-2021-unmaintained)
        public static final int CAN_FRONT_LEFT_DRIVE = 7;
        public static final int CAN_FRONT_LEFT_STEER = 8;
        public static final int CAN_FRONT_LEFT_ENCODER = 4;
        public static final double CAN_FRONT_LEFT_OFFSET = -Math.toRadians(180.0);
        public static final int CAN_FRONT_RIGHT_DRIVE = 3;
        public static final int CAN_FRONT_RIGHT_STEER = 4;
        public static final int CAN_FRONT_RIGHT_ENCODER = 3;
        public static final double CAN_FRONT_RIGHT_OFFSET = -Math.toRadians(180.0);

        public static final int CAN_BACK_LEFT_DRIVE = 5;
        public static final int CAN_BACK_LEFT_STEER = 6;
        public static final int CAN_BACK_LEFT_ENCODER = 2;
        public static final double CAN_BACK_LEFT_OFFSET = -Math.toRadians(180.0);

        public static final int CAN_BACK_RIGHT_DRIVE = 1;
        public static final int CAN_BACK_RIGHT_STEER = 2;
        public static final int CAN_BACK_RIGHT_ENCODER = 1;
        public static final double CAN_BACK_RIGHT_OFFSET = -Math.toRadians(180.0);

        //TODO Used 2023 values (need to change)
        public static final double VAL_MAX_VOLTS = 10;
        public static final double VAL_MAX_CURRENT = 50;

        //For values: units are in meters and radians
        public static final double VAL_TRACK_WIDTH = 0.40125;
        public static final double VAL_WHEEL_BASE = 0.40125;
        //TODO adjust VAL_MAX_VELO
        public static final double VAL_MAX_VELO = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        public static final double VAL_MAX_ACCEL = VAL_MAX_VELO*0.5;
        public static final double VAL_MAX_ANG_VELO = VAL_MAX_VELO/Math.hypot(VAL_TRACK_WIDTH/ 2.0, VAL_WHEEL_BASE/ 2.0);
    }

    public static class TrajectoryFollowing{
        //TODO test values
        public static final double VAL_KP_X = 1;
        public static final double VAL_KP_Y = 1;
        public static final double VAL_KP_THETA= 1;
    }
    public static class Vision {
        public static final double VAL_ANGLE_KP = 0.02;
        public static final double VAL_ANGLE_KI = 0;
        public static final double VAL_ANGLE_KD = 0;
        public static final double VAL_DRIVE_KP = 0.005;
        public static final double VAL_DRIVE_KI = 0;
        public static final double VAL_DRIVE_KD = 0.0005;
        public static final double VAL_LEFT_ANGLE_OFSET = 2;
        public static final double VAL_RIGHT_ANGLE_OFSET = 2;
        public static final double VAL_MOUNT_ANGLE = 21.0;
    }
}