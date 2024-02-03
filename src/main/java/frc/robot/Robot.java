// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

import com.revrobotics.ColorSensorV3;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.sensors.Pigeon2Wrapper;
import frc.team3171.sensors.ThreadedPIDController;
import frc.team3171.HelperFunctions;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.auton.XboxControllerState;
import frc.team3171.controllers.Shooter;

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
  private Shooter shooter;
  private ColorSensorV3 upperFeedSensor, lowerFeedSensor;

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

  // Edge Triggers
  private boolean zeroEdgeTrigger;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Shooter Controller Init
    try {
      shooter = new Shooter();
    } catch (Exception e) {
      e.printStackTrace();
    }

    upperFeedSensor = new ColorSensorV3(Port.kOnboard);
    // lowerFeedSensor = new ColorSensorV3(Port.kMXP);

    // Sensors
    gyro = new Pigeon2Wrapper(GYRO_CAN_ID);
    gyro.reset();

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

    // Global Variable Init
    fieldOrientationChosen = false;

    // Edge Trigger Init
    zeroEdgeTrigger = false;
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

    // Colors Sensor Values
    SmartDashboard.putString("Upper Feed Sensor:", String.format("%s | %d", upperFeedSensor.getColor().toString(), upperFeedSensor.getProximity()));

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
    if (DEBUG) {
      // Operator Controller Values
      SmartDashboard.putString("Left Stick Y", String.format("%.2f", leftStickY));
      SmartDashboard.putString("Right Stick X", String.format("%.2f", rightStickX));
      SmartDashboard.putString("Left Stick Angle", String.format("%.2f\u00B0", leftStickAngle));
      SmartDashboard.putString("Left Stick Velocity", String.format("%.2f", leftStickMagnitude));
      SmartDashboard.putString("Field Adjusted Angle", String.format("%.2f\u00B0", fieldCorrectedAngle));

      // Shooter Values
      SmartDashboard.putNumber("Lower Shooter RPM:", shooter.getLowerShooterVelocity());
      SmartDashboard.putNumber("Upper Shooter RPM:", shooter.getUpperShooterVelocity());
      SmartDashboard.putString("Shooter Tilt Position:", String.format("%.2f", shooter.getShooterTilt()));
      SmartDashboard.putString("Tilt:", String.format("%.2f|%.2f", shooter.test(), shooter.testLock()));

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
  }

  private void driveControlsPeriodic(final XboxControllerState driveControllerState) {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude;
    leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    if (leftStickMagnitude > 1.0) {
      leftStickMagnitude = 1;
    }

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean boostMode = driveControllerState.getLeftBumper() || driveControllerState.getRightBumper();
    if (rightStickX != 0) {
      // Manual turning
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();

      // Quick Turning
      if (driveControllerState.getPOV() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
      } else if (driveControllerState.getYButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(0);
      } else if (driveControllerState.getBButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(90);
      } else if (driveControllerState.getAButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(180);
      } else if (driveControllerState.getXButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(-90);
      }
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    }
  }

  double position = 15;
  boolean pickupEdgeTrigger = false;

  private void operatorControlsPeriodic(final XboxControllerState operatorControllerState) {
    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, operatorControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getLeftY()) / 4;
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, operatorControllerState.getRightX());
    final double rightStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getRightY());

    // shooter.setLowerFeederSpeed(leftStickY);

    if (operatorControllerState.getBButton() && !pickupEdgeTrigger) {
      shooter.setShooterTiltPosition(15);
      shooter.setLowerFeederSpeed(.5);
      shooter.setUpperFeederSpeed(.3);

    } else if (operatorControllerState.getBButton()) {
      if (upperFeedSensor.getColor().toHexString().equals("#8F5A15") || upperFeedSensor.getProximity() > 550) {
        shooter.setUpperFeederSpeed(0);
        shooter.setLowerFeederSpeed(0);
      }
    } else if (pickupEdgeTrigger) {
      shooter.runUpperFeeder(-.15, .1);
    } else {
      shooter.setLowerFeederSpeed(0);
      shooter.setUpperFeederSpeed(rightStickY);

      double shooterSpeed = operatorControllerState.getRightTriggerAxis() - operatorControllerState.getLeftTriggerAxis();
      shooter.setShooterSpeed(shooterSpeed, shooterSpeed / 4);

      if (operatorControllerState.getAButton()) {
        shooter.setShooterTiltPosition(35);
      } else if (operatorControllerState.getYButton()) {
        shooter.setShooterTiltPosition(-35);
      } else {
        if (leftStickY != 0) {
          shooter.setShooterTiltSpeed(leftStickY);
          position = shooter.getAsDouble();
        } else {
          shooter.setShooterTiltPosition(position);
        }
      }
    }

    pickupEdgeTrigger = operatorControllerState.getBButton();

  }

}
