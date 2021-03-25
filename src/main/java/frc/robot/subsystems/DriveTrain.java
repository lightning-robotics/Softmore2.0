// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private static final WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(Constants.DriveConstants.FRONT_RIGHT_MOTOR);
  private static final WPI_TalonSRX backRightMotor = new WPI_TalonSRX(Constants.DriveConstants.BACK_RIGHT_MOTOR);
  private static final WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(Constants.DriveConstants.FRONT_LEFT_MOTOR);
  private static final WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(Constants.DriveConstants.BACK_LEFT_MOTOR);

  AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // distance between the wheels
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.WeaverConstants.kS,
        Constants.WeaverConstants.kV, Constants.WeaverConstants.kA);
  
    PIDController leftPIDController = new PIDController(Constants.WeaverConstants.kP, 0, 0);
    PIDController rightPIDController = new PIDController(Constants.WeaverConstants.kP, 0, 0);
  
    Pose2d pose;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    DriveTrain.frontLeftMotor.follow(DriveTrain.backLeftMotor);
    DriveTrain.backRightMotor.follow(DriveTrain.frontRightMotor);

    DriveTrain.backLeftMotor.setInverted(false);
    DriveTrain.backRightMotor.setInverted(true);
    DriveTrain.frontLeftMotor.setInverted(false);
    DriveTrain.frontRightMotor.setInverted(true);

    DriveTrain.frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    DriveTrain.backLeftMotor.setNeutralMode(NeutralMode.Brake);
    DriveTrain.frontRightMotor.setNeutralMode(NeutralMode.Brake);
    DriveTrain.backRightMotor.setNeutralMode(NeutralMode.Brake);

    backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    backLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);

    gyro.reset();
  }
  
  public void setRightMotorSpeed(double speed) {
    frontRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftMotorSpeed(double speed) {
    backLeftMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getDeadzone(double speed) {
    return (Math.abs(speed) < Constants.DriveConstants.DEADZONE);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public double getVelocity(TalonSRX motor) {
    // getSelectedSensorVelocity returns encoder units per 100 ms
    // returns in meters per second
    return motor.getSelectedSensorVelocity() * (10.0 / 4096) * 2 * Math.PI
        * Units.inchesToMeters(Constants.WeaverConstants.wheelRadius);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    // velocity
    return new DifferentialDriveWheelSpeeds(getVelocity(backLeftMotor), getVelocity(frontRightMotor));
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    backLeftMotor.set(ControlMode.PercentOutput, leftVolts / 12.0);
    frontRightMotor.set(ControlMode.PercentOutput, rightVolts / 12.0);
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getVelocity(backLeftMotor), getVelocity(frontRightMotor));
  }
}
