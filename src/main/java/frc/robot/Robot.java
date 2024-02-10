// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

// REV Imports
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

// Photon Vision Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.models.ShooterShot;
import frc.team3171.models.XboxControllerState;
import frc.team3171.operator.Shooter;
import frc.team3171.sensors.Pigeon2Wrapper;
import frc.team3171.HelperFunctions;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.controllers.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Controllers
  private XboxController driveController, operatorController;

  // Drive Objects
  private SwerveDrive swerveDrive;
  private Pigeon2Wrapper gyro;
  private ThreadedPIDController gyroPIDController;

  // Shooter Objects
  private Shooter shooterController;
  private ColorSensorV3 upperFeedColorSensor;
  private ColorMatch colorMatcher;

  // Photon Vision Objects
  private PhotonCamera frontTargetingCamera, rearTargetingCamera;
  private PhotonCamera frontPickupCamera, rearPickupCamera;

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Selected Auton String
  private boolean selectedAutonType;
  private String selectedAutonMode;

  // Shuffleboard Choosers
  private SendableChooser<Boolean> autonTypeChooser, fieldOrientationChooser;
  private SendableChooser<String> autonModeChooser;

  // Global Variables
  private boolean fieldOrientationChosen;
  private double shooterAtSpeedStartTime;
  private double shooterTiltTargetPosition;

  // Edge Triggers
  private boolean zeroEdgeTrigger;
  private boolean pickupEdgeTrigger;
  private boolean shooterButtonEdgeTrigger;
  private boolean shooterAtSpeedEdgeTrigger;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Shooter Controller Init
    try {
      shooterController = new Shooter();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Sensors
    gyro = new Pigeon2Wrapper(GYRO_CAN_ID);
    gyro.reset();
    upperFeedColorSensor = new ColorSensorV3(Port.kOnboard);
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(RING_COLOR_ONE);

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(gyro.asSupplier(), GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX, true);
    gyroPIDController.start();

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = false;
    autonTypeChooser = new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auton", false);
    autonTypeChooser.addOption("Record Auton", true);
    SmartDashboard.putData("Auton Type", autonTypeChooser);

    // Field Orientation Chooser
    fieldOrientationChooser = new SendableChooser<>();
    fieldOrientationChooser.setDefaultOption("Pick an option", null);
    fieldOrientationChooser.addOption("0\u00B0", false);
    fieldOrientationChooser.addOption("180\u00B0", true);
    SmartDashboard.putData("Field Orientation Chooser", fieldOrientationChooser);
    SmartDashboard.putBoolean("Flipped", false);

    // Auton Modes init
    selectedAutonMode = DEFAULT_AUTON;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(DEFAULT_AUTON, DEFAULT_AUTON);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auton Modes", autonModeChooser);

    // Color Matcher init
    colorMatcher.addColorMatch(RING_COLOR_ONE);
    colorMatcher.setConfidenceThreshold(COLOR_CONFIDENCE);

    // Photon Vision init
    frontTargetingCamera = new PhotonCamera("Arducam_5MP_Camera_Module");

    // Global Variable Init
    fieldOrientationChosen = false;
    shooterAtSpeedStartTime = 0;
    shooterTiltTargetPosition = 0;

    // Edge Triggers Init
    zeroEdgeTrigger = false;
    pickupEdgeTrigger = false;
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
  }

  @Override
  public void robotPeriodic() {
    // Gyro Value
    final double gyroValue = gyroPIDController.getSensorValue();

    // Field Orientation Chooser
    final Boolean fieldOrientationBoolean = fieldOrientationChooser.getSelected();
    // Until a valid option is choosen leave the gyro orientation alone
    if (fieldOrientationBoolean != null && !fieldOrientationChosen) {
      // Prevents the field orientation from being changed until a reboot
      fieldOrientationChosen = true;
      // If the selected option is true then flip the orientation 180 degrees
      if (fieldOrientationBoolean.booleanValue()) {
        gyro.setYaw(Normalize_Gryo_Value(gyroValue + 180));
        SmartDashboard.putBoolean("Flipped", true);
      } else {
        // Else don't flip the field orientation
        SmartDashboard.putBoolean("Flipped", false);
      }
    }

    // Driver Controller Info
    double leftStickX, leftStickY, rightStickX, leftStickAngle, leftStickMagnitude, fieldCorrectedAngle;
    if (driveController.isConnected()) {
      // Get the controller values
      leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getLeftX());
      leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveController.getLeftY());
      rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

      // Calculate the left stick angle and magnitude
      leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
      leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
      if (leftStickMagnitude > 1.0) {
        leftStickMagnitude = 1;
      }

      fieldCorrectedAngle = Normalize_Gryo_Value(leftStickAngle - gyroValue);
    } else {
      leftStickX = 0;
      leftStickY = 0;
      leftStickAngle = 0;
      leftStickMagnitude = 0;
      rightStickX = 0;
      fieldCorrectedAngle = 0;
    }

    // Put the values on Shuffleboard
    SmartDashboard.putString("Gyro", String.format("%.2f\u00B0", gyroValue));

    // Colors Sensor Values
    final int red = upperFeedColorSensor.getBlue(), green = upperFeedColorSensor.getGreen(), blue = upperFeedColorSensor.getBlue();
    SmartDashboard.putString("Upper Feed Sensor:", String.format("[R: %d, G: %d, B: %d] | Prox: %d", red, green, blue, upperFeedColorSensor.getProximity()));
    SmartDashboard.putBoolean("Ring Color Match", colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null);

    // Shooter Info
    SmartDashboard.putBoolean("Shooter At Speed:", shooterController.isBothShootersAtVelocity(SHOOTER_TILT_DESIRED_PERCENT_ACCURACY));
    SmartDashboard.putString("Lower Shooter RPM:",
        String.format("%.2f | %.2f", shooterController.getLowerShooterVelocity(), shooterController.getLowerShooterTargetVelocity()));
    SmartDashboard.putString("Upper Shooter RPM:",
        String.format("%.2f | %.2f", shooterController.getUpperShooterVelocity(), shooterController.getUpperShooterTargetVelocity()));
    SmartDashboard.putString("Shooter Tilt:", String.format("%.2f | %.2f", shooterController.test(), shooterController.testLock()));

    if (DEBUG) {
      // Operator Controller Values
      SmartDashboard.putString("Left Stick Y", String.format("%.2f", leftStickY));
      SmartDashboard.putString("Right Stick X", String.format("%.2f", rightStickX));
      SmartDashboard.putString("Left Stick Angle", String.format("%.2f\u00B0", leftStickAngle));
      SmartDashboard.putString("Left Stick Velocity", String.format("%.2f", leftStickMagnitude));
      SmartDashboard.putString("Field Adjusted Angle", String.format("%.2f\u00B0", fieldCorrectedAngle));

      // Shooter Values
      SmartDashboard.putString("Shooter Tilt Raw:", String.format("%.2f", shooterController.getShooterTiltRaw()));

      // Photon Vision Read Outs
      SmartDashboard.putBoolean("Front Targeting Camera:", frontTargetingCamera.isConnected());
      int targetID = 0;
      double range = 0, yaw = 0, pitch = 0;
      var result = frontTargetingCamera.getLatestResult();
      if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        targetID = bestTarget.getFiducialId();
        range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(bestTarget.getPitch()));
        yaw = bestTarget.getYaw();
      }
      SmartDashboard.putString("Best Target:", String.format("ID: %d | R: %.2f | Y: %.2f | P: %.2f", targetID, range, yaw, pitch));
      SmartDashboard.putString("Tracking Angle:", String.format("%.2f", Normalize_Gryo_Value(gyroValue + yaw)));

      swerveDrive.SmartDashboard();
    }

    // Calibrate Swerve Drive
    final boolean zeroTrigger = driveController.getBackButton() && driveController.getStartButton() && isDisabled();
    if (zeroTrigger && !zeroEdgeTrigger) {
      // Zero the swerve units
      swerveDrive.zero();
      System.out.println("Swerve Drive has been calibrated!");
    }
    zeroEdgeTrigger = zeroTrigger;
  }

  @Override
  public void autonomousInit() {
    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          disabledInit();
          playbackData = null;
          break;
        default:
          AutonRecorder.loadFromFile(autonPlaybackQueue, selectedAutonMode);
          playbackData = autonPlaybackQueue.poll();
          robotControlsInit();
          break;
      }
    }
    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        disabledPeriodic();
        break;
      default:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the controller states
          final XboxControllerState driveControllerState = playbackData.getDriveControllerState();
          final XboxControllerState operatorControllerState = playbackData.getOperatorControllerState();

          // Robot drive controls
          driveControlsPeriodic(driveControllerState);
          operatorControlsPeriodic(operatorControllerState);

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        } else {
          selectedAutonMode = DEFAULT_AUTON;
          disabledInit();
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Reset the robot controls
    robotControlsInit();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    // Get the controller states
    final XboxControllerState driveControllerState = new XboxControllerState(driveController);
    final XboxControllerState operatorControllerState = new XboxControllerState(operatorController);

    // Robot drive controls
    driveControlsPeriodic(driveControllerState);
    operatorControlsPeriodic(operatorControllerState);

    // Auton Recording
    final double autonTimeStamp = Timer.getFPGATimestamp() - autonStartTime;
    if (saveNewAuton && autonTimeStamp <= 15) {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder.addNewData(new AutonRecorderData(autonTimeStamp, driveControllerState, operatorControllerState));
          break;
      }
    }
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    gyroPIDController.disablePID();
    shooterController.disable();

    // Once auton recording is done, save the data to a file, if there is any
    if (saveNewAuton) {
      saveNewAuton = false;
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          autonRecorder.saveToFile(selectedAutonMode);
          break;
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    // Do Nothing
  }

  @Override
  public void testInit() {
    // Do Nothing
  }

  @Override
  public void testPeriodic() {
    // Do Nothing
  }

  private void robotControlsInit() {
    // Reset the drive controller
    swerveDrive.driveInit();
    gyroPIDController.enablePID();
    gyroPIDController.updateSensorLockValue();

    // Global Variable reset
    fieldOrientationChosen = false;
    shooterAtSpeedStartTime = 0;
    shooterTiltTargetPosition = 0;

    // Edge Triggers reset
    zeroEdgeTrigger = false;
    pickupEdgeTrigger = false;
    shooterButtonEdgeTrigger = false;
    shooterAtSpeedEdgeTrigger = false;
  }

  private void driveControlsPeriodic(final XboxControllerState driveControllerState) {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude;
    leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean boostMode = driveControllerState.getXButton();
    if (rightStickX != 0) {
      // Manual turning
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else if (driveControllerState.getAButton()) {
      // April Tag Target Locking
      gyroPIDController.enablePID();

      // Check if the targeting camera is connected, otherwise normally gyro lock
      if (frontTargetingCamera.isConnected()) {
        // Query the latest result from PhotonVision
        final PhotonPipelineResult result = frontTargetingCamera.getLatestResult();
        if (result.hasTargets()) {
          final PhotonTrackedTarget bestTarget = result.getBestTarget();
          final double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(bestTarget.getPitch()));
          // Adjust the gyro lock to point torwards the target
          gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + bestTarget.getYaw()));
        }
      }

      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    } else if (driveControllerState.getBButton()) {
      // TODO Pickup Locking
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();

      // Quick Turning
      if (driveControllerState.getPOV() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
      }
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    }

    // TODO Arm Controls
    if (driveControllerState.getLeftBumper()) {
      // Raise Left Arm
    } else if (driveControllerState.getLeftTriggerAxis() > .02) {
      // Lower Left Arm
    } else {
      // Disable Left Arm
    }

    if (driveControllerState.getRightBumper()) {
      // Raise Right Arm
    } else if (driveControllerState.getRightTriggerAxis() > .02) {
      // Lower Right Arm
    } else {
      // Disable Right Arm
    }
  }

  private void operatorControlsPeriodic(final XboxControllerState operatorControllerState) {
    // Get the needed joystick values after calculating the deadzones
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getLeftY());

    // Get shooter controls
    final boolean button_Pickup = operatorControllerState.getRightTriggerAxis() > .02;
    final boolean button_Reverse_Feed = operatorControllerState.getLeftTriggerAxis() > .02;
    final boolean button_Short_Shot = operatorControllerState.getBButton();
    final boolean button_Normal_Shot = operatorControllerState.getAButton();
    final boolean button_Far_Shot = operatorControllerState.getXButton();
    final boolean button_Yeet_Shot = operatorControllerState.getYButton();
    final boolean button_Shooter = button_Short_Shot || button_Normal_Shot || button_Far_Shot || button_Yeet_Shot;

    // Shooter controls start
    final ShooterShot selectedShot;
    if (button_Short_Shot) {
      // Short shot velocity
      selectedShot = SHOOTER_SHOTS.get("SHORT_SHOT");
    } else if (button_Normal_Shot) {
      // Normal shot velocity
      selectedShot = SHOOTER_SHOTS.get("NORMAL_SHOT");
    } else if (button_Far_Shot) {
      // Far shot velocity
      selectedShot = SHOOTER_SHOTS.get("FAR_SHOT");
    } else if (button_Yeet_Shot) {
      // Yeet shot velocity
      selectedShot = SHOOTER_SHOTS.get("YEET_SHOT");
    } else {
      selectedShot = null;
    }

    // Shooter Control
    if (button_Shooter && !shooterButtonEdgeTrigger) {
      // Shooter Start
      shooterAtSpeedEdgeTrigger = false;
      if (selectedShot != null) {
        shooterTiltTargetPosition = selectedShot.getShooterAngle();
        shooterController.setShooterVelocity(selectedShot.lowerShooterRPM, selectedShot.upperShooterRPM);
      } else {
        shooterController.setShooterSpeed(1);
      }
    } else if (button_Shooter) {
      // Check if the shooter is at speed
      final boolean isShooterAtSpeed = selectedShot == null ? true : shooterController.isBothShootersAtVelocity(SHOOTER_TILT_DESIRED_PERCENT_ACCURACY);
      if (isShooterAtSpeed && !shooterAtSpeedEdgeTrigger) {
        // Get time that shooter first designated at speed
        shooterAtSpeedStartTime = Timer.getFPGATimestamp();
      } else if (isShooterAtSpeed && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + SHOOTER_DESIRED_AT_SPEED_TIME)) {
        // Feed the ball through the shooter
        shooterController.setLowerFeederSpeed(LOWER_FEED_SHOOT_SPEED);
        shooterController.setUpperFeederSpeed(UPPER_FEED_SHOOT_SPEED);
      } else if (colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null) {
        // Back off the ball from the feed sensor
        shooterController.setLowerFeederSpeed(LOWER_FEED_BACKFEED_SPEED);
        shooterController.setUpperFeederSpeed(UPPER_FEED_BACKFEED_SPEED);
      } else {
        // Feeder stopped while shooter gets up to speeed
        shooterController.setLowerFeederSpeed(0);
        shooterController.setUpperFeederSpeed(0);
      }
      shooterAtSpeedEdgeTrigger = isShooterAtSpeed;
    } else {
      // Ring Pickup Controls
      if (button_Pickup && !pickupEdgeTrigger) {
        // Pickup controls start
        shooterTiltTargetPosition = 0;
        shooterController.setShooterSpeed(0);
      } else if (button_Pickup) {
        // Pickup controls while held
        shooterTiltTargetPosition = 0;
        if (HelperFunctions.Within_Percent_Error(shooterController.getShooterTilt(), shooterTiltTargetPosition, SHOOTER_TILT_DESIRED_PERCENT_ACCURACY)) {
          if (colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null) {
            shooterController.setLowerFeederSpeed(0);
            shooterController.setUpperFeederSpeed(0);
          } else {
            shooterController.setLowerFeederSpeed(LOWER_FEED_PICKUP_SPEED);
            shooterController.setUpperFeederSpeed(UPPER_FEED_PICKUP_SPEED);
          }
        } else {
          shooterController.setLowerFeederSpeed(0);
          shooterController.setUpperFeederSpeed(0);
        }
      } else if (pickupEdgeTrigger && (colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null)) {
        // Pickup control when ended
        shooterController.setLowerFeederSpeed(0);
        shooterController.runUpperFeeder(UPPER_FEED_END_SPEED, UPPER_FEED_END_TIME);
      } else if (button_Reverse_Feed) {
        shooterController.setShooterSpeed(SHOOTER_REVERSE_FEED_SPEED);
        shooterController.setLowerFeederSpeed(0);
        if (colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null) {
          shooterController.setUpperFeederSpeed(UPPER_FEED_BACKFEED_SPEED);
        } else {
          shooterController.setUpperFeederSpeed(0);
        }
      } else {
        // Disable both feeders and the shooter
        shooterController.setShooterSpeed(0);
        shooterController.setLowerFeederSpeed(0);
        shooterController.setUpperFeederSpeed(0);
      }
    }

    // Manual Tilt Controls
    if (leftStickY != 0) {
      shooterController.setShooterTiltSpeed(leftStickY / 4);
      shooterTiltTargetPosition = shooterController.getShooterTilt();
    } else {
      shooterController.setShooterTiltPosition(shooterTiltTargetPosition);
    }

    // Edge Trigger Updates
    pickupEdgeTrigger = button_Pickup;
    shooterButtonEdgeTrigger = button_Shooter;
  }

}
