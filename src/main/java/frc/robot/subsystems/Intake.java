// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static WPI_TalonSRX leftMotor = new WPI_TalonSRX(Constants.IntakeConstants.LEFT_INTAKE_MOTOR);
  private static WPI_TalonSRX rightMotor = new WPI_TalonSRX(Constants.IntakeConstants.RIGHT_INTAKE_MOTOR);
  private static WPI_TalonSRX centerMotor = new WPI_TalonSRX(Constants.IntakeConstants.CENTER_INTAKE_MOTOR);
  private static WPI_TalonSRX backMotor = new WPI_TalonSRX(Constants.IntakeConstants.BACK_INTAKE_MOTOR);


  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed != 0 ? speed + 0.1 : speed);
    rightMotor.set(ControlMode.PercentOutput, speed != 0 ? -speed - 0.1 : -speed);
    centerMotor.set(ControlMode.PercentOutput, -speed);
    backMotor.set(ControlMode.PercentOutput, speed != 0 ? -speed + 0.05 : -speed);
  }
}
