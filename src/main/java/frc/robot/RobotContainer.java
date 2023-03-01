package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
    public static final Joystick driver = new Joystick(0);
    // private final PS4Controller PS4 = new PS4Controller(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton lowPos = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton medPos = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton highPos = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton enableVac = new JoystickButton(driver, PS4Controller.Button.kCircle.value);  
    private final JoystickButton armHome = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    private final JoystickButton groundPickUp = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton humanPickUp = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton manualIncreaseArm = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton manualDecreasrArm = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton balanceRobot = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);

    // private final JoystickButton limeLightModeBlink = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
 
    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final ArmSubsystem s_ArmSubsystem = new ArmSubsystem(); // Add Arm Subsystem
    //private final Limelight s_limelight = new Limelight();
    // private final LimelightsubSystem s_limelightsub = new LimelightsubSystem();
    // private final Limelight s_Limelight = new Limelight();

    /* Sendable Chooser and Autonomus Commands - need to work on this */
    private static SendableChooser<Command> autoChooser;
    private final Command m_autoLeft = new SequentialCommandGroup(
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        new adjustArm(98, 30.5, true, true, true),
        new adjustArm(93, 29.5, true, false, true),
        new WaitCommand(0.5),
        new adjustArm(98, 30.5, false, false, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, false, false, false),

        // back up across the line 
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),(new Pose2d(-3, 0, new Rotation2d(0)))),true)
    );
    
    private final Command m_autoCenter =  new SequentialCommandGroup(
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        new adjustArm(98, 30.5, true, true, true),
        new adjustArm(93, 29.5, true, false, true),
        new WaitCommand(0.5),
        new adjustArm(98, 30.5, false, false, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, false, false, false),

        //backing into the charging dock 110 inches, so that the robot is at an angle so that balanceorobtcommand works

        /*new AutoDrive(List.of(
            (new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(0, -2.794, new Rotation2d(0)))),true), */

        new InstantCommand(()-> new BalanceRobotCommand().balanceSwerve())

    );

    private final Command m_autoRight= new SequentialCommandGroup(
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        new adjustArm(98, 30.5, true, true, true),
        new adjustArm(93, 29.5, true, false, true),
        new WaitCommand(0.5),
        new adjustArm(98, 30.5, false, false, true),
        new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, false, false, false),
           
            //back up till blue line 140 inches.
            new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),
                (new Pose2d(-1,0.4318, new Rotation2d(0))),
                (new Pose2d(-3.556, 0.4318, new Rotation2d(0)))),true)
    );

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
              
        // Limelight Testing
        //s_limelightsub.getDistance();
        LimelightHelpers.setStreamMode_Standard("limelight");
        LimelightHelpers.setCameraMode_Driver("limelight");
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Driver Station Left: ", m_autoLeft);
        autoChooser.addOption("Center: ",m_autoCenter);
        autoChooser.addOption("Driver Station Right: ", m_autoRight);
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
        // Vacume
        enableVac.onTrue(new InstantCommand(()-> s_ArmSubsystem.toggleVac()));

        /* Arm */
        //arm1.onTrue(new adjustArm(70,12,true,true,true));
        armHome.onTrue(new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0,false,false,false));
        
        lowPos.onTrue(new SequentialCommandGroup(
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        new adjustArm(47.5,17,true,false,true)
        ));
        // medPos.onTrue(new adjustArm(85, 15.5, false, false, true)); - try this Sequential Command:
        medPos.onTrue(new SequentialCommandGroup(
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
            new adjustArm(85, 13.75, true, true, true),
            new adjustArm(80, 13.25, true, false, true),
            new WaitCommand(0.5),
            new adjustArm(85, 13.75, false, false, true)
        ));
        highPos.onTrue(new SequentialCommandGroup(
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
            new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
            new adjustArm(98, 30.5, true, true, true),
            new adjustArm(93, 29.5, true, false, true),
            new WaitCommand(0.5),
            new adjustArm(98, 30.5, false, false, true)
            ));
        //lowPos.whileFalse(new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0,true,false,false));
       balanceRobot.onTrue(new BalanceRobotCommand());
       groundPickUp.onTrue(new adjustArm(47.5,17,true,true,true));
       groundPickUp.onFalse(new SequentialCommandGroup(
            new adjustArm(60, 17, true, false, true),
            new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0,true,false,false)

        ));

        humanPickUp.onTrue(new adjustArm(85,2,true,true,true));
        humanPickUp.onFalse( new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0,true,false,false)
        );

        
        manualIncreaseArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0.5)));
        manualIncreaseArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));
        manualDecreasrArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(-0.5)));
        manualDecreasrArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));

        
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // limeLightModeBlink.onTrue(new InstantCommand(()-> s_limelight.forceOff()));
        // limeLightModeBlink.onFalse(new InstantCommand(()-> s_limelight.forceOn()));
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
