// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class adjustArm extends SequentialCommandGroup {
  /** Creates a new adjustArm. */
  public adjustArm(double pivotAngle, double rackPosition, boolean vacOnAtStart, boolean vacOnAtEnd, boolean pivotFirst) {
    if (pivotFirst) {
      addCommands(new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(vacOnAtStart)),
          new movePivot(pivotAngle), new moveRack(rackPosition),
          new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(vacOnAtEnd)));
    } else {
      addCommands(new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(vacOnAtStart)),
      new moveRack(rackPosition), new movePivot(pivotAngle),
          new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(vacOnAtEnd)));
    }
  }

  public adjustArm(double pivotAngle, double rackPosition, boolean pivotFirst) {
    if (pivotFirst) {
      addCommands(new movePivot(pivotAngle), new moveRack(rackPosition));
    } else {
      addCommands(new moveRack(rackPosition), new movePivot(pivotAngle));
    }
  }
}
