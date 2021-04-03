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

  public static AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // distance between the wheels
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
    DifferentialDriveOdometry odometry;
  
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.WeaverConstants.kS,
        Constants.WeaverConstants.kV, Constants.WeaverConstants.kA);
  
    PIDController leftPIDController = new PIDController(Constants.WeaverConstants.kP, 0, 0);
    PIDController rightPIDController = new PIDController(Constants.WeaverConstants.kP, 0, 0);
  
    Pose2d pose;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    frontLeftMotor.setInverted(false);
    backLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    backLeftMotor.setSensorPhase(true);
    frontRightMotor.setSensorPhase(true);

    frontLeftMotor.follow(DriveTrain.backLeftMotor);
    backRightMotor.follow(DriveTrain.frontRightMotor);

    backLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);

    gyro.reset();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void resetEncoders() {
    backLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
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

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }
  
  public double getVelocity(TalonSRX motor) {
    // getSelectedSensorVelocity returns encoder units per 100 ms
    // returns in meters per second
    return getMetersPerSec(motor.getSelectedSensorVelocity());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    // velocity
    return new DifferentialDriveWheelSpeeds(
        getVelocity(backLeftMotor), 
        getVelocity(frontRightMotor));
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setOutput(double leftVolts, double rightVolts) {
    backLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
    // double leftMult = (Math.ceil(Math.abs(leftVolts)) / 12 + 1) * 12;
    // double rightMult = (Math.ceil(Math.abs(rightVolts)) / 12 + 1) * 12;
    // double mult = Math.max(leftMult, rightMult);

    // if (leftVolts / mult > 1) System.out.println((leftVolts / mult) + " " + leftVolts);
    // backLeftMotor.set(ControlMode.PercentOutput, leftVolts/mult);
    // frontRightMotor.set(ControlMode.PercentOutput, rightVolts/mult); 
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public double getDistance(double ticks) {
    double circumference = Math.PI * Units.inchesToMeters(Constants.WeaverConstants.wheelRadius * 2);
    return ticks * circumference / 4096.;
  }

  public double getMetersPerSec(double ticks) {
    return getDistance(ticks * 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(gyro.getRotation2d(), 
    getDistance(backLeftMotor.getSelectedSensorPosition()),
    getDistance(frontRightMotor.getSelectedSensorPosition()));
  }
}
