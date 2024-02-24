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

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;

// REV Imports
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Photon Vision Imports
import org.photonvision.targeting.PhotonTrackedTarget;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.models.ShooterShot;
import frc.team3171.models.XboxControllerState;
import frc.team3171.operator.Shooter;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.controllers.ThreadedPIDController;
import frc.team3171.controllers.VisionController;
import static frc.team3171.HelperFunctions.Deadzone;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;
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
  private Pigeon2 gyro;
  private ThreadedPIDController gyroPIDController;

  // Shooter Objects
  private Shooter shooterController;
  private ColorSensorV3 upperFeedColorSensor;
  private ColorMatch colorMatcher;

  // Linear Actuators
  private CANSparkMax leftAcuator, rightAcuator;

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

  // Vision Controller
  private VisionController visionController;

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

    // Linear Acuators
    leftAcuator = new CANSparkMax(LEFT_ACUATOR_CAN_ID, MotorType.kBrushed);
    rightAcuator = new CANSparkMax(RIGHT_ACUATOR_CAN_ID, MotorType.kBrushed);

    // Sensors
    gyro = new Pigeon2(GYRO_CAN_ID, "canivore");
    gyro.reset();
    upperFeedColorSensor = new ColorSensorV3(Port.kOnboard);
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(RING_COLOR_ONE);

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(() -> Normalize_Gryo_Value(gyro.getAngle()), GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX, false);
    gyroPIDController.setMinValue(-180);
    gyroPIDController.setMaxValue(180);
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

    // Vision Controller Init
    visionController = new VisionController();
    // visionController.shuffleboardTabInit("FRONT_TARGETING_CAMERA", "Front Cameras");
    // visionController.shuffleboardTabInit("REAR_TARGETING_CAMERA", "Rear Cameras");

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
      leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getLeftX());
      leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveController.getLeftY());
      rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

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
    SmartDashboard.putString("Gyro", String.format("%.2f\u00B0 | %.2f\u00B0", gyroValue, gyroPIDController.getSensorLockValue()));

    // Colors Sensor Values
    final int red = upperFeedColorSensor.getBlue(), green = upperFeedColorSensor.getGreen(), blue = upperFeedColorSensor.getBlue();
    SmartDashboard.putString("Upper Feed Sensor:", String.format("[R: %d, G: %d, B: %d] | Prox: %d", red, green, blue, upperFeedColorSensor.getProximity()));
    SmartDashboard.putBoolean("Ring Color Match", colorMatcher.matchColor(upperFeedColorSensor.getColor()) != null);

    // Shooter Info
    SmartDashboard.putBoolean("Shooter At Speed:", shooterController.isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION));
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
    final double gyroValue = gyroPIDController.getSensorValue();

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude;
    leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean boostMode = driveControllerState.getXButton();
    final boolean targetLocking = driveControllerState.getAButton();
    final boolean pickupLocking = driveControllerState.getBButton();
    final boolean robotOffGround = Math.abs(gyro.getRoll().getValueAsDouble()) > 5 || Math.abs(gyro.getPitch().getValueAsDouble()) > 5;
    SmartDashboard.putBoolean("Off Ground:", robotOffGround);
    if (rightStickX != 0 || robotOffGround) {
      // Manual turning
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else if (targetLocking) {
      // April Tag Target Locking
      gyroPIDController.enablePID();

      final PhotonTrackedTarget frontTargetingCameraBestTarget = visionController.getCameraBestTarget("FRONT_TARGETING_CAMERA");
      if (frontTargetingCameraBestTarget != null) {
        // Adjust the gyro lock to point torwards the target
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + frontTargetingCameraBestTarget.getYaw()));
      }

      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    } else if (pickupLocking) {
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

      final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? (closeEnough ? 0 : gyroPIDController.getPIDValue()) : 0, boostMode);
    }

    // Arm Controls
    final boolean raiseLeftArm = driveControllerState.getLeftBumper();
    final boolean lowerLeftArm = Deadzone(.02, driveControllerState.getLeftTriggerAxis()) != 0;
    if (raiseLeftArm) {
      // Raise Left Arm
      leftAcuator.set(1);
    } else if (lowerLeftArm) {
      // Lower Left Arm
      leftAcuator.set(-1);
    } else {
      // Disable Left Arm
      leftAcuator.set(0);
    }

    final boolean raiseRightArm = driveControllerState.getRightBumper();
    final boolean lowerRightArm = Deadzone(.02, driveControllerState.getRightTriggerAxis()) != 0;
    if (raiseRightArm) {
      // Raise Right Arm
      rightAcuator.set(1);
    } else if (lowerRightArm) {
      // Lower Right Arm
      rightAcuator.set(-1);
    } else {
      // Disable Right Arm
      rightAcuator.set(0);
    }

    if (raiseLeftArm || lowerLeftArm || raiseRightArm || lowerRightArm) {
      // Force the shooter down
      shooterTiltTargetPosition = 70;
    }
  }

  private void operatorControlsPeriodic(final XboxControllerState operatorControllerState) {
    final double gyroValue = gyroPIDController.getSensorValue();

    // Get the needed joystick values after calculating the deadzones
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getLeftY());

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
    final boolean shooterTiltWithinRange = Math
        .abs(Get_Gyro_Displacement(shooterController.getShooterTilt(), shooterTiltTargetPosition)) < SHOOTER_TILT_ALLOWED_DEVIATION;
    if (button_Shooter && !shooterButtonEdgeTrigger) {
      // Shooter Start
      shooterAtSpeedEdgeTrigger = false;
      if (selectedShot != null) {
        shooterTiltTargetPosition = gyroValue < -135 || gyroValue > 45 ? selectedShot.getShooterAngle() : -selectedShot.getShooterAngle();
        shooterController.setShooterVelocity(selectedShot.lowerShooterRPM, selectedShot.upperShooterRPM);
      } else {
        shooterController.setShooterSpeed(1);
      }
    } else if (button_Shooter) {
      shooterTiltTargetPosition = gyroValue < -135 || gyroValue > 45 ? selectedShot.getShooterAngle() : -selectedShot.getShooterAngle();
      // Check if the shooter is at speed
      final boolean isShooterAtSpeed = selectedShot == null ? true : shooterController.isBothShootersAtVelocity(SHOOTER_TILT_ALLOWED_DEVIATION);
      if (isShooterAtSpeed && !shooterAtSpeedEdgeTrigger) {
        // Get time that shooter first designated at speed
        shooterAtSpeedStartTime = Timer.getFPGATimestamp();
      } else if (shooterTiltWithinRange && isShooterAtSpeed && (Timer.getFPGATimestamp() >= shooterAtSpeedStartTime + SHOOTER_DESIRED_AT_SPEED_TIME)) {
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
        if (shooterTiltWithinRange) {
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
        shooterTiltTargetPosition = gyroValue < -45 || gyroValue > 135 ? -40 : 40;
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
