package frc.lib.util;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class SwerveConstants {
    public final double wheelDiameter_m;
    public final double wheelCircumference_m;
    public final double angleGearRatio_m;
    public final double driveGearRatio_m;
    public final double angleKP_m;
    public final double angleKI_m;
    public final double angleKD_m;
    public final double angleKF_m;
    public final boolean driveMotorInvert_m;
    public final boolean angleMotorInvert_m;
    public final boolean canCoderInvert_m;

    public SwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert){
        this.wheelDiameter_m = wheelDiameter;
        this.wheelCircumference_m = wheelDiameter * Math.PI;
        this.angleGearRatio_m = angleGearRatio;
        this.driveGearRatio_m = driveGearRatio;
        this.angleKP_m = angleKP;
        this.angleKI_m = angleKI;
        this.angleKD_m = angleKD;
        this.angleKF_m = angleKF;
        this.driveMotorInvert_m = driveMotorInvert;
        this.angleMotorInvert_m = angleMotorInvert;
        this.canCoderInvert_m = canCoderInvert;
    }

    /** Swerve Drive Specialties - MK4i Module*/
    public static SwerveConstants SDSMK4i(){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);
        double driveGearRatio = (6.75 / 1.0);

        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;
        return new SwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    }
}
