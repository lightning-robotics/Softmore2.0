// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.WeaverConstants;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final Challenge1 m_autoCommand = new Challenge1("Blue_path");

  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Intake intake = new Intake();
  public static final Limelight limelight = new Limelight();

  public static final Command tankDrive = new TankDrive();
  public static final IntakeIn intakeIn = new IntakeIn();

  public static XboxController driver = new XboxController(Constants.PortConstants.ROBOT_DRIVE_CONTROLLER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // Trajectory trajectory = getTrajectory(Constants.WeaverConstants.BARREL1_JSON);
    System.out.println("Made the trajectory");
    // if (limelight.isVisible()) {
    //   // get the red path
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.WeaverConstants.SEARCH_ARED_JSON);
    //   try {
    //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //   } catch (IOException e) {
    //     e.printStackTrace();
    //   }
    // } else {
    //     // get the blue path
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.WeaverConstants.INTAKE_BLUE);
        try {
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
          e.printStackTrace();
        }
    // }

      return getTrajectoryCommand(trajectory).andThen(() -> driveTrain.setOutput(0, 0));
      // return getBouncePath();
  }

  public static Command getTrajectoryCommand(Trajectory trajectory) {
    driveTrain.resetEncoders();
    driveTrain.zeroHeading();

    RamseteCommand command = new RamseteCommand(
      trajectory,
      driveTrain::getPose,
      new RamseteController(2.0, 0.7),
      driveTrain.getFeedforward(),
      driveTrain.getKinematics(),
      driveTrain::getSpeeds,
      driveTrain.getLeftPIDController(),
      driveTrain.getRightPIDController(),
      driveTrain::setOutput,
      driveTrain
    );
    System.out.println("Made the command");
    System.out.println(command);

    driveTrain.resetOdometry(trajectory.getInitialPose());

    return command;
  }

  public static Trajectory getTrajectory(String path) {
    Trajectory trajectory = new Trajectory();
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
  
      e.printStackTrace();
    }
    return trajectory;
  }

  public static Command getBouncePath() {
    
    Trajectory trajectory1 = getTrajectory(Constants.WeaverConstants.BARREL1_JSON);
    Trajectory trajectory2 = getTrajectory(Constants.WeaverConstants.BARREL2_JSON);
    Trajectory trajectory3 = getTrajectory(Constants.WeaverConstants.BARREL3_JSON);
    Trajectory trajectory4 = getTrajectory(Constants.WeaverConstants.BARREL4_JSON);

    return getTrajectoryCommand(trajectory1)
            .andThen(() -> getTrajectoryCommand(trajectory2))
            .andThen(() -> getTrajectoryCommand(trajectory3))
            .andThen(() -> getTrajectoryCommand(trajectory4))
            .andThen(() -> driveTrain.setOutput(0, 0));
  }

  public static double getControllerAxis(XboxController controller, int axis) {
    return controller.getRawAxis(axis);
  }
}
