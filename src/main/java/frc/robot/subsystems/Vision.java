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
     * @param tv_m limelight has a valid target, 1 is yes & 0 is no
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

        /**
     * 0 for on, 1 for off
     * @param camMode
     */
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

        /**
     * Fetch the vertical offset from crosshair to target (ty)
     * @return ty
     */
    public double calcTy(){
        //get ty from the network table
        return networkTable_m.getEntry("ty").getDouble(0.0);
    }
    /**
     * @return The distance from the Limelight to the target
     */
    public double calcGoalDistance(){
        //calculate the x distance to the goal
        return 209.0 / Math.tan(Math.toRadians(VAL_MOUNT_ANGLE + calcTy()));
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
