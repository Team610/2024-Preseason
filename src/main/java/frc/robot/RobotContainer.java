package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static Joystick driver = new Joystick(0);
    public static Vision visionInst_s = new Vision();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton spinButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton travelButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton visionButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton resetButton = new JoystickButton(driver, XboxController.Button.kA.value);
    //button to end Spin or Travel command
    private final JoystickButton cancelButton = new JoystickButton(driver, XboxController.Button.kA.value);


    /* Subsystems */
    private static Swerve s_Swerve;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve =Swerve.getInstance();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        spinButton.onTrue(new Spin(s_Swerve, 135, () -> -driver.getRawAxis(translationAxis), ()-> -driver.getRawAxis(strafeAxis), () -> false, cancelButton));
        travelButton.onTrue(new Travel(s_Swerve, 0, 0, () -> false, cancelButton));
        visionButton.onTrue(new T_Vision_Drive(s_Swerve, visionInst_s));
        resetButton.onTrue(new T_Swerve_Reset(s_Swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Move(s_Swerve);
    }
}
