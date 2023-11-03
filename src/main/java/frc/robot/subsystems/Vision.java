package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.Subsystem610;

import static frc.robot.Constants.Vision.*;

public class Vision extends Subsystem610 {
    private static Vision visionInst_s;

    /**
     * @param ledMode_m led, 0 is on & 1 is off
     * @param tv_m limelight has a valid target, 0 is on & 1 is off I'm guessing
     */
    private int ledMode_m;
    private int tv_m;
    private NetworkTable networkTable_m;

    private Shuffleboard visionTab;




    public static Vision getInstance(){
        if (visionInst_s == null){
            visionInst_s = new Vision();
        }
        return visionInst_s;
    }

    private Vision(){
        super("Limelight");

        ledMode_m=0;
        
        ShuffleboardTab visionTab = Shuffleboard.getTab("test");

        visionTab.add("Limelight", new HttpCamera("limelight","http://10.6.10.87:5800/stream.mjpg "))
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(3,3);


        networkTable_m = NetworkTableInstance.getDefault().getTable("limelight");

        networkTable_m.getEntry("ledMode").setNumber(ledMode_m);
    }

    public void setCamMode(int camMode){
        networkTable_m.getEntry("camMode").setNumber(camMode);
    }

    /**
     * 0 for on, 1 for off
     * @param ledMode
     */
    public void setLedMode(int ledMode){
        networkTable_m.getEntry("camMode").setNumber(ledMode);
    }

    public double getGoalDistance(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 
    
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 
    
        // distance from the target to the floor
        double goalHeightInches = 60.0; 
    
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    // Required Method
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        
    }

    //Required Method
    @Override
    public void addToDriveTab(ShuffleboardTab tab) {
        // TODO Auto-generated method stub
        
    }
    
}
