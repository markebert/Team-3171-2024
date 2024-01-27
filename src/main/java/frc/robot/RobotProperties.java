package frc.robot;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.drive.SwerveUnitConfig.SwerveUnitConfigBuilder;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Options **/
        public static final boolean DEBUG = true;
        public static final boolean SWERVE_DIRECTION_DEBUG = false;
        public static final String PID_LOG_ADDRESS = "10.31.71.201";

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
        public static final int LOWER_SHOOTER_CAN_ID = 20, UPPER_SHOOTER_CAN_ID = 21, SHOOTER_TILT_CAN_ID = 30;
        public static final int LOWER_FEEDER_CAN_ID = 20, UPPER_FEEDER_CAN_ID = 21;

        /** Shooter Variables **/
        public static final boolean LOWER_SHOOTER_INVERTED = false, UPPER_SHOOTER_INVERTED = true, SHOOTER_TILT_INVERTED = false;
        public static final int LOWER_SHOOTER_VELOCITY = 1400, UPPER_SHOOTER_VELOCITY = 3800; // High shot
        public static final int LOWER_SHOOTER_SHORT_VELOCITY = 500, UPPER_SHOOTER_SHORT_VELOCITY = 1925; // Low shot
        public static final int LOWER_SHOOTER_YEET_VELOCITY = 3650, UPPER_SHOOTER_YEET_VELOCITY = 5350; // Yeet shot
        public static final double DESIRED_PERCENT_ACCURACY = .08, DESIRED_PERCENT_ACCURACY_YEET = .05,
                        DESIRED_AT_SPEED_TIME = .06, DESIRED_AT_SPEED_TIME_SHORT = .2; // Accuracy Settings

        /** Pickup Variables **/
        public static final int PICKUP_ARM_MAX_CURRENT = 90;
        public static final double REVERSE_PICKUP_SPEED = -.5;
        public static final double PICKUP_SPEED = .55;

        /** Feeder Variables **/
        public static final boolean LOWER_FEEDER_INVERTED = false, UPPER_FEEDER_INVERTED = true;
        public static final double SHOOTER_LOWER_FEED_SPEED = .1, SHOOTER_UPPER_FEED_SPEED = .25;
        public static final double REVERSE_LOWER_FEEDER_SPEED = -.75, REVERSE_UPPER_FEEDER_SPEED = -.75;
        public static final double LOWER_FEEDER_SPEED = .3, LOWER_FEEDER_SPEED_SLOW = .2;
        public static final double UPPER_FEEDER_SPEED = .2, UPPER_FEEDER_BACKFEED_SPEED = -.4;
        public static final double LOWER_FEED_END_SPEED = .15, LOWER_FEED_END_TIME = .2;
        public static final double UPPER_FEED_END_SPEED = -.3, UPPER_FEED_END_TIME = .2;

        /** PID Properties **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = -.005, SLEW_KI = -.0004, SLEW_KD = .035, SLEW_KF = 0, SLEW_PID_MIN = -1, SLEW_PID_MAX = 1;
        public static final double SHOOTER_KP = .01, SHOOTER_KI = .0002, SHOOTER_KD = .0001, SHOOTER_KF = 0;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}