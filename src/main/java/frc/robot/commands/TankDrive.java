// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveTrain.setLeftMotorSpeed(0);
    RobotContainer.driveTrain.setRightMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double driverAxis = RobotContainer.getControllerAxis(RobotContainer.driver, Constants.PortConstants.ROBOT_DRIVE_YAXIS);
    double driverAxis2 = RobotContainer.getControllerAxis(RobotContainer.driver, Constants.PortConstants.ROBOT_DRIVE_XAXIS);

    double varSpeed = 0.95;

    driverAxis = Math.min(Math.abs(driverAxis), varSpeed) * (driverAxis/Math.abs(driverAxis));
    driverAxis2 = Math.min(Math.abs(driverAxis2), varSpeed) * (driverAxis2/Math.abs(driverAxis2));

    if (RobotContainer.driveTrain.getDeadzone(driverAxis2)) 
      driverAxis2 = 0;
    if (RobotContainer.driveTrain.getDeadzone(driverAxis))
      driverAxis = 0;
    

  
    RobotContainer.driveTrain.setRightMotorSpeed((driverAxis / -2) - (driverAxis2 / 2));
    RobotContainer.driveTrain.setLeftMotorSpeed((driverAxis / -2) + (driverAxis2 / 2));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
