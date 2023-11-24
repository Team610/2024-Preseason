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
        public static final double VAL_MOUNT_ANGLE = 45.0;
    }
}