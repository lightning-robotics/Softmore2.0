// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry ta = table.getEntry("ta");
  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getXAngle() {
    return tx.getDouble(0.0);
  }

  public double getYAngle() {
    return ty.getDouble(0.0);
  }

  public boolean isVisible() {
    return tv.getDouble(0.0) == 1;
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }
}
