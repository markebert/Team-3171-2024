package frc.robot;

// Java Imports
import java.util.HashMap;

import edu.wpi.first.math.util.Units;
// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.drive.SwerveUnitConfig.SwerveUnitConfigBuilder;
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
        public static final double MAX_DRIVE_SPEED = .5, MAX_ROTATION_SPEED = .6;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = true;
        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV, 2, MOTOR_TYPE.REV, 3, ENCODER_TYPE.REV).build();
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV, 4, MOTOR_TYPE.REV, 5, ENCODER_TYPE.REV).build();
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV, 6, MOTOR_TYPE.REV, 7, ENCODER_TYPE.REV).build();
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.REV, 8, MOTOR_TYPE.REV, 9, ENCODER_TYPE.REV).build();

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 10;
        public static final int LOWER_SHOOTER_CAN_ID = 17, UPPER_SHOOTER_CAN_ID = 16, SHOOTER_TILT_CAN_ID = 15;
        public static final int LOWER_FEEDER_MASTER_CAN_ID = 11, LOWER_FEEDER_FOLLOWER_CAN_ID = 12;
        public static final int UPPER_FEEDER_MASTER_CAN_ID = 13, UPPER_FEEDER_FOLLOWER_CAN_ID = 14;

        /** Shooter Variables **/
        public static final boolean LOWER_SHOOTER_INVERTED = true, UPPER_SHOOTER_INVERTED = true, SHOOTER_TILT_INVERTED = false;
        public static final double SHOOTER_TILT_DESIRED_PERCENT_ACCURACY = .1; // Shooter Tilt Accuracy Settings
        public static final double SHOOTER_DESIRED_PERCENT_ACCURACY = .05; // Shooter Veloctity Accuracy Settings
        public static final double SHOOTER_DESIRED_AT_SPEED_TIME = .2; // Shooter Veloctity Time Window Settings
        public static final double SHOOTER_REVERSE_FEED_SPEED = -.5;
        public static final HashMap<String, ShooterShot> SHOOTER_SHOTS = new HashMap<>() {
                {
                        put("SHORT_SHOT", new ShooterShot(60, 2500, 2500));
                        put("NORMAL_SHOT", new ShooterShot(45, 3150, 400));
                        put("FAR_SHOT", new ShooterShot(35, 4000, 4000));
                        put("YEET_SHOT", new ShooterShot(60, 6300, 6300));
                }
        };

        public static final double SHOOTER_ENCODER_ZERO_POSITION = 25;
        public static final double SHOOTER_ENCODER_MIN_POSITION = -75, SHOOTER_ENCODER_MAX_POSITION = 75;

        /** Feeder Variables **/
        public static final boolean LOWER_FEEDER_INVERTED = false, UPPER_FEEDER_INVERTED = false;
        public static final double LOWER_FEED_PICKUP_SPEED = .5, UPPER_FEED_PICKUP_SPEED = .25;
        public static final double UPPER_FEED_END_SPEED = -.1, UPPER_FEED_END_TIME = .1;
        public static final double LOWER_FEED_SHOOT_SPEED = 0, UPPER_FEED_SHOOT_SPEED = .4;
        public static final double LOWER_FEED_BACKFEED_SPEED = 0, UPPER_FEED_BACKFEED_SPEED = -.075;

        public static final double SHOOTER_LOWER_FEED_SPEED = .1, SHOOTER_UPPER_FEED_SPEED = .25;
        public static final double REVERSE_LOWER_FEEDER_SPEED = 0, REVERSE_UPPER_FEEDER_SPEED = -.2;
        public static final double LOWER_FEEDER_SPEED = .3, LOWER_FEEDER_SPEED_SLOW = .2;

        /** Photon Vision Constants **/
        public final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(14);
        public final double TARGET_HEIGHT_METERS = Units.inchesToMeters(56.125);
        public final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(22.5); // Angle between horizontal and the camera.

        /** PID Variables **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = -.005, SLEW_KI = -.0004, SLEW_KD = .035, SLEW_KF = 0, SLEW_PID_MIN = -1, SLEW_PID_MAX = 1;
        public static final double SHOOTER_KP = .00025, SHOOTER_KI = .0004, SHOOTER_KD = -.002, SHOOTER_KF = 0, SHOOTER_MIN = -1, SHOOTER_MAX = 1;
        public static final double TILT_KP = .007, TILT_KI = .003, TILT_KD = -.01, TILT_KF = 0, TILT_MIN = -.75, TILT_MAX = .75;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}