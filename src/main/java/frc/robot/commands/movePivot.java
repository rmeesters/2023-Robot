// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class movePivot extends CommandBase {
  private ArmSubsystem arm = RobotContainer.s_ArmSubsystem;
  private double degrees;

  /** Creates a new movePivot. */
  public movePivot(double degrees) {
    this.degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.goPivotToPosition(degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (degrees - 2 < arm.getPivotAngle() && arm.getPivotAngle() < degrees + 2) {
      return true;
    } else {
      return false;
    }
  }
}
