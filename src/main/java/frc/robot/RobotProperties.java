package frc.robot;

// Java Imports
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// FRC Imports
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.drive.SwerveUnitConfig.SwerveUnitConfigBuilder;
import frc.team3171.models.PhotonCameraConfig;
import frc.team3171.models.ShooterShot;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Options **/
        public static final boolean DEBUG = true;
        public static final boolean SWERVE_DIRECTION_DEBUG = false;
        public static final String PID_LOG_ADDRESS = "10.31.71.202";

        /** Drive Variables **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = 1, MAX_ROTATION_SPEED = .6;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = true;
        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV_SPARKFLEX, 2, MOTOR_TYPE.REV_SPARKMAX, 3,
                        ENCODER_TYPE.REV, 3).build();
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV_SPARKFLEX, 4, MOTOR_TYPE.REV_SPARKMAX, 5,
                        ENCODER_TYPE.REV, 1).build();
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV_SPARKFLEX, 6, MOTOR_TYPE.REV_SPARKMAX, 7,
                        ENCODER_TYPE.REV, 2).build();
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV_SPARKFLEX, 8, MOTOR_TYPE.REV_SPARKMAX, 9,
                        ENCODER_TYPE.REV, 0).build();

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 10;
        public static final int LOWER_SHOOTER_CAN_ID = 17, UPPER_SHOOTER_CAN_ID = 16, SHOOTER_TILT_CAN_ID = 15;
        public static final int LOWER_FEEDER_MASTER_CAN_ID = 11, LOWER_FEEDER_FOLLOWER_CAN_ID = 12;
        public static final int UPPER_FEEDER_MASTER_CAN_ID = 13, UPPER_FEEDER_FOLLOWER_CAN_ID = 14;
        public static final int LEFT_ACUATOR_CAN_ID = 18, RIGHT_ACUATOR_CAN_ID = 19;

        /** Shooter Variables **/
        public static final boolean LOWER_SHOOTER_INVERTED = true, UPPER_SHOOTER_INVERTED = true, SHOOTER_TILT_INVERTED = false;
        public static final double SHOOTER_TILT_ALLOWED_DEVIATION = 5; // Shooter Tilt Accuracy Settings
        public static final double SHOOTER_ALLOWED_PERCENT_ERROR = .05; // Shooter Veloctity Accuracy Settings
        public static final double SHOOTER_DESIRED_AT_SPEED_TIME = .5; // Shooter Veloctity Time Window Settings
        public static final double SHOOTER_REVERSE_FEED_SPEED = -.5;
        public static final HashMap<String, ShooterShot> SHOOTER_SHOTS = new HashMap<>() {
                {
                        put("SHORT_SHOT", new ShooterShot(45, 300, 2000));
                        put("NORMAL_SHOT", new ShooterShot(40, 2500, 3000));
                        put("FAR_SHOT", new ShooterShot(59, 4000, 4000));
                        put("YEET_SHOT", new ShooterShot(69, 5500, 5500));
                }
        };

        public static final int SHOOTER_TILT_ID = 4;
        public static final double SHOOTER_TILT_ZERO_POSITION = 11.25;
        public static final double SHOOTER_TILT_MIN_POSITION = -70, SHOOTER_TILT_MAX_POSITION = 70;

        /** Feeder Variables **/
        public static final boolean LOWER_FEEDER_INVERTED = true, UPPER_FEEDER_INVERTED = false;
        public static final double LOWER_FEED_PICKUP_SPEED = .5, UPPER_FEED_PICKUP_SPEED = .25;
        public static final double UPPER_FEED_END_SPEED = -.1, UPPER_FEED_END_TIME = .1;
        public static final double LOWER_FEED_SHOOT_SPEED = 0, UPPER_FEED_SHOOT_SPEED = .8;
        public static final double LOWER_FEED_BACKFEED_SPEED = 0, UPPER_FEED_BACKFEED_SPEED = -.075;
        public static final Color RING_COLOR_ONE = new Color(143, 90, 21);
        public static final double COLOR_CONFIDENCE = .85;

        /** Photon Vision Constants **/
        public static final HashMap<String, PhotonCameraConfig> PHOTON_CAMERAS_CONFIGS = new HashMap<>() {
                {
                        put("FRONT_TARGETING_CAMERA", new PhotonCameraConfig("FRONT_TARGETING_CAMERA", Units.inchesToMeters(14), Units.degreesToRadians(22.5)));
                        put("REAR_TARGETING_CAMERA", new PhotonCameraConfig("REAR_TARGETING_CAMERA", Units.inchesToMeters(14), Units.degreesToRadians(22.5)));
                        put("FRONT_PICKUP_CAMERA", new PhotonCameraConfig("FRONT_PICKUP_CAMERA", Units.inchesToMeters(0), Units.degreesToRadians(0)));
                        put("REAR_PICKUP_CAMERA", new PhotonCameraConfig("REAR_PICKUP_CAMERA", Units.inchesToMeters(0), Units.degreesToRadians(0)));
                }
        };

        public static final HashMap<Integer, Alliance> APRILTAG_FIELD_COLOR = new HashMap<>() {
                {
                        // Blue Feed Station
                        put(1, Alliance.Blue);
                        put(2, Alliance.Blue);
                        // Red Speaker
                        put(3, Alliance.Red);
                        put(4, Alliance.Red); // More Centered
                        // Red Low Goal
                        put(5, Alliance.Red);
                        // Blue Low Goal
                        put(6, Alliance.Blue);
                        // Blue Speaker
                        put(7, Alliance.Blue); // More Centered
                        put(8, Alliance.Blue);
                        // Red Feed Station
                        put(9, Alliance.Red);
                        put(10, Alliance.Red);
                        // Red Center Goals
                        put(11, Alliance.Red);
                        put(12, Alliance.Red);
                        put(13, Alliance.Red);
                        // Blue Center Goals
                        put(14, Alliance.Blue);
                        put(15, Alliance.Blue);
                        put(16, Alliance.Blue);

                }
        };

        public static final AprilTagFieldLayout AprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        /** PID Variables **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = -.005, SLEW_KI = -.0004, SLEW_KD = .035, SLEW_KF = 0, SLEW_PID_MIN = -1, SLEW_PID_MAX = 1;
        public static final double SHOOTER_KP = .00025, SHOOTER_KI = .0004, SHOOTER_KD = -.002, SHOOTER_MIN = -1, SHOOTER_MAX = 1;
        public static final double TILT_KP = .02, TILT_KI = .0005, TILT_KD = -.02, TILT_MIN = -.75, TILT_MAX = .75;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}