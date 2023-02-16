package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private final Joystick driver = new Joystick(0);
    // private final PS4Controller PS4 = new PS4Controller(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton limeLightModeBlink = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton enableVac = new JoystickButton(driver, PS4Controller.Button.kCircle.value);  
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem(); // Add Arm Subsystem
    private final Limelight s_limelight = new Limelight();
    private final LimelightsubSystem s_limelightsub = new LimelightsubSystem();
    // private final Limelight s_Limelight = new Limelight();

    /* Sendable Chooser and Autonomus Commands - need to work on this */
    private static SendableChooser<Command> autoChooser;
    private final Command m_autoOne = new station1Auto(s_Swerve, s_ArmSubsystem);
    private final Command m_autoTwo = new exampleAuto(s_Swerve); 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        s_limelightsub.getDistance();
        
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Auto One", m_autoOne);
        autoChooser.addOption("Auto Two", m_autoTwo);
        SmartDashboard.putData("Auto mode", autoChooser);   
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        enableVac.onTrue(new InstantCommand(()-> s_ArmSubsystem.enableVac(true)));
        enableVac.onFalse(new InstantCommand(() -> s_ArmSubsystem.enableVac(false)));  //Not sure this is required

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        limeLightModeBlink.onTrue(new InstantCommand(()-> s_limelight.forceOff()));
        limeLightModeBlink.onFalse(new InstantCommand(()-> s_limelight.forceOn()));
        //limeLightModeBlink.onTrue(new Limelight.forceBlink()); 
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return autoChooser.getSelected();
    }
}
