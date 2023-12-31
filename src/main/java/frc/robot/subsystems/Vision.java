package frc.robot.subsystems;

import static frc.robot.Constants.Vision.VAL_ANGLE_KD;
import static frc.robot.Constants.Vision.VAL_ANGLE_KI;
import static frc.robot.Constants.Vision.VAL_ANGLE_KP;
import static frc.robot.Constants.Vision.VAL_LEFT_ANGLE_OFSET;
import static frc.robot.Constants.Vision.VAL_RIGHT_ANGLE_OFSET;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private static Vision visionInst_s;
    private int ledMode_m;
    private int tv_m;
    private NetworkTable networkTable_m;
    private PIDController pidAngle_m;
    private int conePosition_m;
    private double angleSetPoint_m;
    private int distanceSetPoint_m;
    public ShuffleboardTab visionTab_m;
    public PIDController pidX_s = new PIDController(0.06, 0.05, 0);
    public PIDController pidY_s = new PIDController(0.06, 0.05, 0);
    public PIDController pidAng_s = new PIDController(0.032, 0, 0);
    Swerve swerveInst_s = Swerve.getInstance();
    private boolean actInPos = false;
    int isInPosCnt = 0;
    int counter = 0;

    public static Vision getInstance() {
        if (visionInst_s == null)
            visionInst_s = new Vision();
        return visionInst_s;
    }

    public Vision() {
        isInPosCnt = 0;
        actInPos = false;
        ledMode_m = 0;
        conePosition_m = 0;
        angleSetPoint_m = 0;
        distanceSetPoint_m = 372;

        visionTab_m = Shuffleboard.getTab("test");

        visionTab_m.add("Limelight", new HttpCamera("limelight", "http://10.6.10.98:5800/stream.mjpg"))
                .withWidget(BuiltInWidgets.kCameraStream)
                .withPosition(0, 0)
                .withSize(3, 3);

        // make sure the pipeline team number is set to 610
        networkTable_m = NetworkTableInstance.getDefault().getTable("limelight");

        networkTable_m.getEntry("ledMode").setNumber(ledMode_m);
        // setCamMode(0);
        pidAngle_m = new PIDController(VAL_ANGLE_KP, VAL_ANGLE_KI, VAL_ANGLE_KD);
        pidX_s.setSetpoint(0); // ! this should be fine
        pidAng_s.setSetpoint(0);
        pidY_s.setSetpoint(-10);
        pidX_s.setTolerance(1);
        pidY_s.setTolerance(1);
        pidAng_s.setTolerance(3);
        setPipeline(0);
    }

    public int getLedMode() {
        return ledMode_m;
    }

    public void setCamMode(int camMode) {
        networkTable_m.getEntry("camMode").setNumber(camMode);
    }

    public void resetPID() {
        pidAngle_m.reset();
        pidX_s.reset();
        pidY_s.reset();
    }

    /**
     * 0 for on, 1 for off
     * 
     * @param ledMode
     */
    public void setLedMode(int ledMode) {
        networkTable_m.getEntry("ledMode").setNumber(ledMode);
    }

    public int getConePosition() {
        return conePosition_m;
    }

    public void setConePosition(int newPosition) {
        conePosition_m = newPosition;
        if (conePosition_m == 0) {
            angleSetPoint_m = 0;
        }
        // cone is to the left
        else if (conePosition_m == 1) {
            angleSetPoint_m = VAL_LEFT_ANGLE_OFSET;
        }
        // cone is to the right
        else {
            angleSetPoint_m = VAL_RIGHT_ANGLE_OFSET;
        }
    }

    /**
     * Fetch the horizontal offset from crosshair to target (tx)
     * 
     * @return tx
     */
    public double calcTx() {
        // ternary operator to return the horizontal angle if a valid target is detected
        return networkTable_m.getEntry("tx").getDouble(0.0);
    }

    /**
     * Fetch the vertical offset from crosshair to target (ty)
     * 
     * @return ty
     */
    public double calcTy() {
        return tv_m == 0 ? 0 : networkTable_m.getEntry("ty").getDouble(0.0);
    }

    public double calcTa() {
        return tv_m == 0 ? 0 : networkTable_m.getEntry("ta").getDouble(0.0);
    }

    public double[] calcBotpose() {
        return tv_m == 0 ? new double[] { 0, 0, 0, 0, 0, 0 }
                : networkTable_m.getEntry("t6r_ts").getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    }

    /**
     * @return The distance from the Limelight to the target
     */
    public double calcDistance() {
        return tv_m == 0 ? 0 : 209.0 / Math.tan(Math.toRadians(21 + calcTy()));
    }

    public boolean checkDistance() {
        return calcDistance() > 0 ? calcDistance() - distanceSetPoint_m < 10 : false;
    }

    public boolean isReady() {
        InetAddress limelightIP;
        try {
            limelightIP = InetAddress.getByName("10.6.10.98");
            boolean reachable = limelightIP.isReachable(3000);
            if (reachable) {
                return true;
            } else {
                return false;
            }
        } catch (UnknownHostException e) {
            return false;
        } catch (IOException e) {
            return false;
        }
    }

    public void turnOn() {
        // setCamMode(0);
        setLedMode(3);
        // Timer.delay(2);
    }

    public void turnOff() {
        // setCamMode(1);
        setLedMode(1);
    }

    public int getTv() {
        return tv_m;
    }

    public boolean isInPos() {
        return actInPos;
    }

    public void setInPos() {
        actInPos = false;
        counter = 0;
    }

    public void drive(double timeStart, double currentTime) {
        // if(isReady()) {
        // if(currentTime - timeStart > 2){
        // actInPos = true;
        // swerveInst_s.stop();
        // }else{
        // swerveInst_s.drive(new Translation2d(0, 0), 0, true, true);
        // }
        // }
        if (getTv() == 1) {
            double degrees = swerveInst_s.getYaw().getDegrees() % 360;
            if (degrees > 180) {
                degrees -= 360;
            }
            if (degrees < -180) {
                degrees += 360;
            }
            double rSpeed;
            if (Math.abs(degrees) < 3) {
                rSpeed = 0;
            } else {
                // rSpeed = 0;
                rSpeed = pidAng_s.calculate(degrees % 360);
            }
            // double rSpeed = 0;
            double xSpeed = pidX_s.calculate(calcTx());
            double ySpeed = pidY_s.calculate(calcTy());
            // double xSpeed = 0;
            // double ySpeed = 0;
            Translation2d translation = new Translation2d(ySpeed, xSpeed);
            if (degrees > 180) {
                degrees -= 360;
            }
            if (degrees < -180) {
                degrees += 360;
            }
            SmartDashboard.putNumber("Degree", degrees);
            SmartDashboard.putNumber("AngleError", pidAng_s.getPositionError());
            SmartDashboard.putNumber("Y Error", pidY_s.getPositionError());
            SmartDashboard.putNumber("X Error", pidX_s.getPositionError());
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("rSpeed", rSpeed);

            swerveInst_s.drive(translation, rSpeed, true, true);
            if (Math.abs(swerveInst_s.getYaw().getDegrees() % 360) < 3 && Math.abs(calcTy() + 10) < 1)
                isInPosCnt++; // TODO tune???
            else
                isInPosCnt = 0;

            if (isInPosCnt > 20) {
                actInPos = true;
                swerveInst_s.stop();
            }
        } else {
            if (currentTime - timeStart > 1) {
                actInPos = true;
                swerveInst_s.stop();
            } else {
                swerveInst_s.drive(new Translation2d(0, 0), 0, true, true);
            }
        }
    }

    public double[] getAimPID() {
        double steeringAdjust = pidAngle_m.calculate(calcTx(), angleSetPoint_m);

        return new double[] { -steeringAdjust, steeringAdjust };

    }

    public void setPipeline(int lineNum) {
        networkTable_m.getEntry("pipeline").setNumber(lineNum);
    }

    public void writeDashboard() {
        SmartDashboard.putNumber("calcTx", Math.round(calcTx() * 1e5) / 1e5);
        SmartDashboard.putNumber("Aimed", getTv());
        SmartDashboard.putNumber("calcTy", calcTy());
        SmartDashboard.putNumber("Distance", calcDistance());
    }

    @Override
    public void periodic() {
        tv_m = (int) networkTable_m.getEntry("tv").getDouble(0.0);
        writeDashboard();
    }
}