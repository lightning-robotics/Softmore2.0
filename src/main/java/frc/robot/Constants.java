// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *  
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class DriveConstants {
        public static final int FRONT_RIGHT_MOTOR = 4;
        public static final int BACK_RIGHT_MOTOR = 5;
        public static final int FRONT_LEFT_MOTOR = 8;
        public static final int BACK_LEFT_MOTOR = 14;

        public static final double DEADZONE = 0.3;


    }

    public final class IntakeConstants {
        public static final int LEFT_INTAKE_MOTOR = 16;
        public static final int RIGHT_INTAKE_MOTOR = 10;
        public static final int CENTER_INTAKE_MOTOR = 6;
        public static final int BACK_INTAKE_MOTOR = 3;

        public static final double INTAKE_SPEED = 0.2;
    }

    public static final class PortConstants {
        public static final int ROBOT_DRIVE_CONTROLLER = 0;
        public static final int ROBOT_DRIVE_YAXIS = 1;
        public static final int ROBOT_DRIVE_XAXIS = 0;
        public static final int ROBOT_DRIVE_XAXIS_2 = 4;
    }

    public static final class WeaverConstants {
        public static final String SEARCH_ARED_JSON = "paths/SearchARed.wpilib.json";
        public static final String SEARCH_ABLUE_JSON = "paths/SearchABue.wpilib.json";

        public static final String SEARCH_BRED_JSON = "paths/SearchBRed.wpilib.json";
        public static final String SEARCH_BBLUE_JSON = "paths/SearchBBlue.wpilib.json";

        public static final String BARREL_JSON = "paths/Barrel.wpilib.json";
        public static final String BOUNCE_JSON = "paths/Bounce.wpilib.json";
        public static final String SALMON_JSON = "paths/Salmon.wpilib.json";

        public static final String FAKE_PATH = "paths/Test1.wpilib.json";

        public static final double wheelRadius = 3;
        // found using robot characterization
        public static final double kS = 0.829;
        public static final double kV = 3.04;
        public static final double kA = 0.676;
        public static final double kP = 2;

        // both are per second
        public static final double maxVelocity = 2;
        public static final double maxAcceleration = 2;
    }
}
