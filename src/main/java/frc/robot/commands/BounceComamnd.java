// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BounceComamnd extends CommandBase {
  /** Creates a new BounceComamnd. */

  Trajectory trajectory1 = RobotContainer.getTrajectory(Constants.WeaverConstants.BARREL1_JSON);
  Trajectory trajectory2 = RobotContainer.getTrajectory(Constants.WeaverConstants.BARREL2_JSON);
  Trajectory trajectory3 = RobotContainer.getTrajectory(Constants.WeaverConstants.BARREL3_JSON);
  Trajectory trajectory4 = RobotContainer.getTrajectory(Constants.WeaverConstants.BARREL4_JSON);

  public BounceComamnd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
