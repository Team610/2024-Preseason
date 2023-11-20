package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Subsystem610;

import static frc.robot.Constants.Vision.*;

public class Vision extends Subsystem610 {
    private static Vision visionInst_s;



    private int ledMode_m;
    private int tv_m;
    private int distanceSetPoint_m;
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

        distanceSetPoint_m = 372;
        
        ShuffleboardTab visionTab = Shuffleboard.getTab("test");

        visionTab.add("Limelight", new HttpCamera("limelight","http://10.6.10.87:5800/stream.mjpg "))
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(3,3);
        
        visionTab.add("ledMode",ledMode_m)
            .withPosition(4,0)
            .withSize(2, 2);
            
        visionTab.add("camMode",tv_m)
            .withPosition(7, 7)
            .withSize(2,2);


        networkTable_m = NetworkTableInstance.getDefault().getTable("limelight");

        networkTable_m.getEntry("ledMode").setNumber(ledMode_m);

    }

    public int getLedMode(){
        return ledMode_m;
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

    /**
     * Fetch the horizontal offset from crosshair to target (tx)
     * @return tx
     */
    public double calcTx(){ 
        // ternary operator to return the horizontal angle if a valid target is detected
        return networkTable_m.getEntry("tx").getDouble(0.0);
    }
    
    /**
     * Fetch the vertical offset from crosshair to target (ty)
     * @return ty
     */
    public double calcTy(){
        //get ty from the network table
        return tv_m == 0 ? 0 : networkTable_m.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return The distance from the Limelight to the target
     */
    public double calcTargetDistance(){
        //calculate the x distance to the goal
        return tv_m == 0 ? 0 : 209.0 / Math.tan(Math.toRadians(VAL_MOUNT_ANGLE + calcTy()));
    }

    public boolean checkTargetDistance(){
        return calcTargetDistance() > 0 ? calcTargetDistance() - distanceSetPoint_m < 10 : false;
    }

    public void writeDashboard(){
        SmartDashboard.putNumber("distance", Math.round(calcTargetDistance() * 1e5)/ 1e5);
    }
    public void periodic(){
        tv_m = (int)networkTable_m.getEntry("tv").getDouble(0.0);
        writeDashboard();
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
