// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Swerve {

      public static final double DPADSPEED = 0.2; 

      // public static final int pigeonID = 1;
      public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- //Changed to True for testing 0129

      public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
          COTSFalconSwerveConstants.GNOMES(COTSFalconSwerveConstants.driveGearRatios.GNOMES);

      /* Drivetrain Constants */
      public static final double trackWidth = Units.inchesToMeters(22.625); //TODO: This must be tuned to specific robot - RM Tuned for 2023 Robot 0114
      public static final double wheelBase = Units.inchesToMeters(25.4375); //TODO: This must be tuned to specific robot - RM Tuned for 2023 Robot 0114
      public static final double wheelCircumference = chosenModule.wheelCircumference;

      /* Swerve Kinematics 
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
       public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

      /* Module Gear Ratios */
      public static final double driveGearRatio = chosenModule.driveGearRatio;
      public static final double angleGearRatio = chosenModule.angleGearRatio;

      /* Motor Inverts */
      public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    //   public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

      /* Angle Encoder Invert */
      public static final boolean canCoderInvert = chosenModule.canCoderInvert;

      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = 25;
      public static final int anglePeakCurrentLimit = 40;
      public static final double anglePeakCurrentDuration = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveContinuousCurrentLimit = 35;
      public static final int drivePeakCurrentLimit = 60;
      public static final double drivePeakCurrentDuration = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
       * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      /* Angle Motor PID Values */
      public static final double angleKP = chosenModule.angleKP;
      public static final double angleKI = chosenModule.angleKI;
      public static final double angleKD = chosenModule.angleKD;
      public static final double angleKF = chosenModule.angleKF;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.4; //TODO: This must be tuned to specific robot
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values 
       * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
      public static final double driveKV = (1.51 / 12);
      public static final double driveKA = (0.27 / 12);

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
      /** Radians per Second */
      public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

      /* Neutral Modes */
      public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
      public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */ //front left
      public static final class Mod0 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 10;
          public static final int canCoderID = 1;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
          public static final   boolean driveMotorInvert = false;
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
      }

      /* Front Right Module - Module 1 */ //front right
      public static final class Mod1 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 11;
          public static final int canCoderID = 2;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
          public static final   boolean driveMotorInvert = true;
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
      }
      
      /* Back Left Module - Module 2 */ //backleft
      public static final class Mod2 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 8;
          public static final int angleMotorID = 12;
          public static final int canCoderID = 3;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
          public static final   boolean driveMotorInvert = false;
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
      }

      /* Back Right Module - Module 3 */ //backright
      public static final class Mod3 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID =9 ;
          public static final int angleMotorID = 13;
          public static final int canCoderID = 4;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
          public static final   boolean driveMotorInvert = true;
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
      }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 2;
      public static final double kPYController = 2;
      public static final double kPThetaController = 2;
  
      /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ArmConstants {
    public static final double pivotBottomAngle = 37.5;
    public static final int armPivot = 15; //adjust for CAN ID
    public static final int armRack = 16;  //adjust for CAN ID
    public static final int airSupplyCAN = 17;
    public static final int pivotEncoder = 5;

    public static final int loopIDX = 0;
    public static final int timeoutMS = 0;

    public static final double armPivotKP = 0.4; //TODO: This must be tuned to specific robot
    public static final double armPivotKI = 0.0;
    public static final double armPivotKD = 0.0;  
    public static final double armPivotKF = 0.0;

    public static final double armRackKP = 0.3; //TODO old: 0.4: This must be tuned to specific robot
    public static final double armRackKI = 0.0;
    public static final double armRackKD = 0.0;  
    public static final double armRackKF = 0.0;

  }
}
