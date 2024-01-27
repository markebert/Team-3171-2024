package frc.team3171.drive;

// Java Imports
import java.util.function.DoubleSupplier;
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.HelperFunctions;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.sensors.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements DoubleSupplier, RobotProperties {

    // Motor Controllers
    private final MOTOR_TYPE driveMotorType, slewMotorType;
    private final MotorController driveMotor, slewMotor;

    // Absolute Encoder
    private final ENCODER_TYPE selectedEncoderType;
    private final CANcoder absoluteEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;
    private final ConcurrentLinkedQueue<String> slewPIDData;

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     * 
     * @param driveInverted
     *            The config settings for the swerve unit.
     */
    public SwerveUnit(final SwerveUnitConfig swerveUnitConfig) {
        // Init the drive motor
        driveMotorType = swerveUnitConfig.getDriveMotorType();
        switch (driveMotorType) {
            case CTRE:
                driveMotor = new TalonFX(swerveUnitConfig.getDriveMotorCANID(), swerveUnitConfig.getCANBUS());
                ((TalonFX) driveMotor).setInverted(swerveUnitConfig.isDriveMotorInverted());
                ((TalonFX) driveMotor).setNeutralMode(NeutralModeValue.Brake);
                break;
            default:
                driveMotor = new CANSparkMax(swerveUnitConfig.getDriveMotorCANID(), MotorType.kBrushless);
                ((CANSparkMax) driveMotor).setInverted(swerveUnitConfig.isDriveMotorInverted());
                ((CANSparkMax) driveMotor).setIdleMode(IdleMode.kBrake);
                break;
        }

        // Init the slew motor
        slewMotorType = swerveUnitConfig.getSlewMotorType();
        switch (slewMotorType) {
            case CTRE:
                slewMotor = new TalonFX(swerveUnitConfig.getSlewMotorCANID(), swerveUnitConfig.getCANBUS());
                ((TalonFX) slewMotor).setInverted(swerveUnitConfig.isSlewMotorInverted());
                ((TalonFX) slewMotor).setNeutralMode(NeutralModeValue.Brake);
                break;
            default:
                slewMotor = new CANSparkMax(swerveUnitConfig.getSlewMotorCANID(), MotorType.kBrushless);
                ((CANSparkMax) slewMotor).setInverted(swerveUnitConfig.isSlewMotorInverted());
                ((CANSparkMax) slewMotor).setIdleMode(IdleMode.kBrake);
                break;
        }

        // Init the absolute position encoder used for the slew angle
        selectedEncoderType = swerveUnitConfig.getAbsoluteEncoderType();
        switch (selectedEncoderType) {
            case CTRE:
                absoluteEncoder = new CANcoder(swerveUnitConfig.getAbsoluteEncoderCANID(), swerveUnitConfig.getCANBUS());
                CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
                absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                absoluteEncoderConfiguration.MagnetSensor.SensorDirection = swerveUnitConfig.isAbsoluteEncoderInverted()
                        ? SensorDirectionValue.CounterClockwise_Positive
                        : SensorDirectionValue.Clockwise_Positive;
                absoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                absoluteEncoder = null;
                break;
        }

        // Init the queue for pid data, if enabled
        slewPIDData = swerveUnitConfig.isLogPIDData() ? new ConcurrentLinkedQueue<String>() : null;

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this::getAsDouble, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, true);
        // slewPIDController.start(20, true, slewPIDData);
        slewPIDController.start(20, true, slewPIDData);

        // Init the global variables
        startingAngle = 0;
    }

    /**
     * Sets the drive motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the drive motor to.
     */
    public void setDriveSpeed(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).set(speed);
                break;
            default:
                ((CANSparkMax) driveMotor).set(speed);
                break;
        }
    }

    /**
     * Returns the current drive motor speed.
     * 
     * @return The current drive motor speed, from -1.0 to 1.0.
     */
    public double getDriveSpeed() {
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).get();
            default:
                return ((CANSparkMax) driveMotor).get();
        }
    }

    /**
     * Sets whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param inverted
     *            Whether or not the direction of the drive motor need to be
     *            inverted.
     */
    public void setDriveInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).setInverted(inverted);
                break;
            default:
                ((CANSparkMax) driveMotor).setInverted(inverted);
                break;
        }
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     * 
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getInverted();
            default:
                return ((CANSparkMax) driveMotor).getInverted();
        }
    }

    /**
     * Sets the slew motor to the desired angle.
     * 
     * @param angle
     *            The angle, from -180.0 to 180.0, to set the slew motor to.
     */
    public void updateSlewAngle(final double angle) {
        // Update the target angle of slew motor PID controller
        slewPIDController.updateSensorLockValueWithoutReset(angle);

        // Update Slew Motor Speed
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).set(slewPIDController.getPIDValue());
                break;
            default:
                ((CANSparkMax) slewMotor).set(slewPIDController.getPIDValue());
                break;
        }
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated
     * target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).set(slewPIDController.getPIDValue());
                break;
            default:
                ((CANSparkMax) slewMotor).set(slewPIDController.getPIDValue());
                break;
        }
    }

    /**
     * Sets the slew motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the slew motor to.
     */
    public void setSlewSpeed(final double speed) {
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).set(speed);
                break;
            default:
                ((CANSparkMax) slewMotor).set(speed);
                break;
        }
    }

    /**
     * Returns the raw value of the {@link MotorController} integrated encoder.
     * 
     * @return The raw value from the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderValue() {
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getPosition().getValueAsDouble();
            default:
                return ((CANSparkMax) driveMotor).getEncoder().getPosition();
        }
    }

    /**
     * Returns the velocity of the {@link MotorController} integrated encoder. If the drive motor type is of
     * {@link MOTOR_TYPE.CTRE}, then the encoder 2048 ticks per revolution and the return units of the velocity is in ticks
     * per 100ms. If the drive motor is of {@link MOTOR_TYPE.REV} then it returns the RPM of the motor.
     * 
     * @return The velocity, in ticks per 100ms or RPM of the
     *         {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderVelocity() {
        switch (driveMotorType) {
            case CTRE:
                return ((TalonFX) driveMotor).getVelocity().getValueAsDouble();
            default:
                return ((CANSparkMax) driveMotor).getEncoder().getVelocity();
        }
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (selectedEncoderType) {
            case CTRE:
                mappedEncoderAngle = HelperFunctions.Map(absoluteEncoder.getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                mappedEncoderAngle = HelperFunctions.Map(((CANSparkMax) slewMotor).getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition(), 0, 1,
                        0, 360);
                break;
        }
        return Normalize_Gryo_Value(mappedEncoderAngle - startingAngle);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewTargetAngle() {
        return slewPIDController.getSensorLockValue();
    }

    /**
     * Returns the current velocity of the {@link CANCoder}.
     * 
     * @return The velocity, in degrees per second.
     */
    public double getSlewVelocity() {
        final double slewVelocity;
        // Get the absolute encoder value based on encoder type
        switch (selectedEncoderType) {
            case CTRE:
                slewVelocity = absoluteEncoder.getVelocity().getValueAsDouble();
                break;
            default:
                switch (slewMotorType) {
                    case CTRE:
                        // If theres no cancoder and not a REV motor then get the value from the TalonFX integrated encoder
                        slewVelocity = ((TalonFX) slewMotor).getVelocity().getValueAsDouble();
                        break;
                    default:
                        // Assumes the encoder is wired into the slew motor spark max
                        slewVelocity = ((CANSparkMax) slewMotor).getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getVelocity();
                        break;
                }
                break;
        }
        return slewVelocity;
    }

    public void enable() {
        slewPIDController.enablePID();
    }

    /**
     * Disables the drive and slew {@link TalonFX} motors.
     */
    public void disable() {
        switch (driveMotorType) {
            case CTRE:
                ((TalonFX) driveMotor).disable();
                break;
            default:
                ((CANSparkMax) driveMotor).disable();
                break;
        }
        switch (slewMotorType) {
            case CTRE:
                ((TalonFX) slewMotor).disable();
                break;
            default:
                ((CANSparkMax) slewMotor).disable();
                break;
        }
        slewPIDController.disablePID();
    }

    @Override
    public double getAsDouble() {
        return getSlewAngle();
    }

    public void zeroModule(final double slewOffset) {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (selectedEncoderType) {
            case CTRE:
                mappedEncoderAngle = HelperFunctions.Map(absoluteEncoder.getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                mappedEncoderAngle = HelperFunctions.Map(((CANSparkMax) slewMotor).getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition(), 0, 1,
                        0, 360);
                break;
        }
        startingAngle = Normalize_Gryo_Value(mappedEncoderAngle - slewOffset);
    }

    public void zeroModule() {
        zeroModule(0);
    }

    public double getSlewOffset() {
        return startingAngle;
    }

    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

    public ConcurrentLinkedQueue<String> getSlewPIDData() {
        return slewPIDData;
    }

}