package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// FRC Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

// REV Imports
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.HelperFunctions;
import frc.team3171.sensors.ThreadedPIDController;

/**
 * @author Mark Ebert
 */
public class Shooter implements DoubleSupplier, RobotProperties {

    // Motor Controllers
    private final CANSparkFlex lowerShooterMotor, upperShooterMotor;
    private final CANSparkMax lowerFeederMotorMaster, lowerFeederMotorFollower, upperFeederMotorMaster, upperFeederMotorFollower, shooterTiltMotor;
    // private final SparkMaxPIDController pickupArmPIDController;

    // PID Controllers
    private final SparkPIDController lowerShooterPIDController, upperShooterPIDController;
    private final ThreadedPIDController shooterTiltPIDController;
    private volatile int lowerShooterTargetRPM, upperShooterTargetRPM;
    private volatile double shooterTiltTargetPosition;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean lowerFeederExecutorActive, upperFeederExecutorActive;

    /**
     * Constructor
     * 
     * @throws Exception
     *             Throws a new exception if there are an invalid amount of
     *             motors in the feederCANIDArray.
     */
    public Shooter() throws Exception {
        // Init all of the motors
        lowerShooterMotor = new CANSparkFlex(LOWER_SHOOTER_CAN_ID, MotorType.kBrushless);
        upperShooterMotor = new CANSparkFlex(UPPER_SHOOTER_CAN_ID, MotorType.kBrushless);
        lowerFeederMotorMaster = new CANSparkMax(LOWER_FEEDER_MASTER_CAN_ID, MotorType.kBrushless);
        lowerFeederMotorFollower = new CANSparkMax(LOWER_FEEDER_FOLLOWER_CAN_ID, MotorType.kBrushless);
        upperFeederMotorMaster = new CANSparkMax(UPPER_FEEDER_MASTER_CAN_ID, MotorType.kBrushless);
        upperFeederMotorFollower = new CANSparkMax(UPPER_FEEDER_FOLLOWER_CAN_ID, MotorType.kBrushless);
        shooterTiltMotor = new CANSparkMax(SHOOTER_TILT_CAN_ID, MotorType.kBrushless);

        // Set if any motors need to be inverted
        lowerFeederMotorMaster.setInverted(LOWER_FEEDER_INVERTED);
        upperFeederMotorMaster.setInverted(UPPER_FEEDER_INVERTED);

        lowerFeederMotorFollower.follow(lowerFeederMotorMaster);
        upperFeederMotorFollower.follow(upperFeederMotorMaster);

        // Configure the velocity closed loop values
        lowerShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.setInverted(LOWER_SHOOTER_INVERTED);
        lowerShooterPIDController = lowerShooterMotor.getPIDController();
        lowerShooterPIDController.setP(SHOOTER_KP);
        lowerShooterPIDController.setI(SHOOTER_KI);
        lowerShooterPIDController.setD(SHOOTER_KD);
        lowerShooterPIDController.setFF(SHOOTER_KF);
        lowerShooterPIDController.setOutputRange(SHOOTER_MIN, SHOOTER_MAX);
        // lowerShooterPIDController.setSmartMotionMaxVelocity(2000, 0);
        // lowerShooterPIDController.setSmartMotionMinOutputVelocity(0, 0);
        // lowerShooterPIDController.setSmartMotionMaxAccel(1500, 0);
        lowerShooterMotor.burnFlash();
        lowerShooterTargetRPM = 0;

        upperShooterMotor.restoreFactoryDefaults();
        upperShooterMotor.setInverted(UPPER_SHOOTER_INVERTED);
        upperShooterPIDController = upperShooterMotor.getPIDController();
        upperShooterPIDController.setP(SHOOTER_KP);
        upperShooterPIDController.setI(SHOOTER_KI);
        upperShooterPIDController.setD(SHOOTER_KD);
        upperShooterPIDController.setFF(SHOOTER_KF);
        upperShooterPIDController.setOutputRange(SHOOTER_MIN, SHOOTER_MAX);
        // upperShooterPIDController.setSmartMotionMaxVelocity(2000, 0);
        // upperShooterPIDController.setSmartMotionMinOutputVelocity(0, 0);
        // upperShooterPIDController.setSmartMotionMaxAccel(1500, 0);
        upperShooterMotor.burnFlash();
        upperShooterTargetRPM = 0;

        shooterTiltMotor.restoreFactoryDefaults();
        shooterTiltMotor.setInverted(SHOOTER_TILT_INVERTED);
        // shooterTiltPidController.setSmartMotionMaxVelocity(2000, 0);
        // shooterTiltPidController.setSmartMotionMinOutputVelocity(0, 0);
        // shooterTiltPidController.setSmartMotionMaxAccel(1500, 0);
        shooterTiltMotor.burnFlash();
        shooterTiltPIDController = new ThreadedPIDController(this::getAsDouble, TILT_KP, TILT_KI, TILT_KD, TILT_MIN, TILT_MAX, true);
        shooterTiltPIDController.start(true);

        // Initialize the executor service for concurrency
        executorService = Executors.newFixedThreadPool(2);
        executorLock = new ReentrantLock(true);

        // Initialize the AtomicBooleans to control the thread executors
        lowerFeederExecutorActive = new AtomicBoolean(false);
        upperFeederExecutorActive = new AtomicBoolean(false);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param lowerShooterSpeed
     *            The speed, from -1.0 to 1.0, to set the lower
     *            shooter motor to.
     * @param upperShooterSpeed
     *            The speed, from -1.0 to 1.0, to set the upper
     *            shooter motor to.
     */
    public void setShooterSpeed(final double lowerShooterSpeed, final double upperShooterSpeed) {
        lowerShooterTargetRPM = 0;
        upperShooterTargetRPM = 0;
        lowerShooterMotor.set(lowerShooterSpeed);
        upperShooterMotor.set(upperShooterSpeed);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param shooterSpeed
     *            The speed, from -1.0 to 1.0, to set the all of the
     *            shooter motors to.
     */
    public void setShooterSpeed(final double shooterSpeed) {
        setShooterSpeed(shooterSpeed, shooterSpeed);
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param lowerShooterTargetRPM
     *            The RPM to set the lower shooter motor to.
     * @param upperShooterTargetRPM
     *            The RPM to set the upper shooter motor to.
     */
    public void setShooterVelocity(final int lowerShooterTargetRPM, final int upperShooterTargetRPM) {
        /**
         * First check if either desired RPM is 0, if so lets the electronic brake
         * handle the slow done rather then the PID Controller.
         */
        if (lowerShooterTargetRPM == 0) {
            lowerShooterMotor.set(0);
        } else if (this.lowerShooterTargetRPM != lowerShooterTargetRPM) {
            lowerShooterPIDController.setReference(lowerShooterTargetRPM, ControlType.kVelocity);
        }
        this.lowerShooterTargetRPM = lowerShooterTargetRPM;
        if (upperShooterTargetRPM == 0) {
            upperShooterMotor.set(0);
        } else if (this.upperShooterTargetRPM != upperShooterTargetRPM) {
            upperShooterPIDController.setReference(upperShooterTargetRPM, ControlType.kVelocity);
        }
        this.upperShooterTargetRPM = upperShooterTargetRPM;
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param shooterRPM
     *            The RPM to set the all of the shooter motors to.
     */
    public void setShooterVelocity(final int shooterRPM) {
        setShooterVelocity(shooterRPM, shooterRPM);
    }

    /**
     * Returns the velocity of the lower shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the lower shooter motor.
     */
    public double getLowerShooterVelocity() {
        return lowerShooterMotor.getEncoder().getVelocity();
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error
     * margin.
     * 
     * @param percentError
     *            The percent error, from 0.0 to 1.0 with 1.0 being
     *            equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isLowerShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double error = Math.abs(lowerShooterTargetRPM - getLowerShooterVelocity());
        final double acceptableError = error * percentError;
        return error < acceptableError;
    }

    /**
     * Returns the velocity of the upper shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterVelocity() {
        return upperShooterMotor.getEncoder().getVelocity();
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error
     * margin.
     * 
     * @param percentError
     *            The percent error, from 0.0 to 1.0 with 1.0 being
     *            equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isUpperShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double error = Math.abs(upperShooterTargetRPM - getUpperShooterVelocity());
        final double acceptableError = error * percentError;
        return error < acceptableError;
    }

    /**
     * Returns the target velocity of the lower shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The target RPM of the lower shooter motor.
     */
    public double getLowerShooterTargetVelocity() {
        return lowerShooterTargetRPM;
    }

    /**
     * Returns the target velocity of the upper shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterTargetVelocity() {
        return upperShooterTargetRPM;
    }

    /**
     * Returns the speed of the lower shooter motor in percent.
     * 
     * @return The speed of the lower shooter motor, from -1.0 to 1.0.
     */
    public double getLowerShooterSpeed() {
        return lowerShooterMotor.get();
    }

    /**
     * Returns the speed of the upper shooter motor.
     * 
     * @return The speed of the upper shooter motor in percent, from -1.0 to 1.0.
     */
    public double getUpperShooterSpeed() {
        return upperShooterMotor.get();
    }

    /**
     * Returns if the velocity of the both shooter motors is within the
     * provided percent error margin.
     * 
     * @param percentError
     *            The percent error, from 0.0 to 1.0 with 1.0 being
     *            equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent
     *         error, false otherwise.
     */
    public boolean isBothShootersAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double upperError = Math.abs(upperShooterTargetRPM - getLowerShooterVelocity());
        final double lowerError = Math.abs(lowerShooterTargetRPM - getLowerShooterVelocity());
        final double upperAcceptableError = upperError * percentError;
        final double lowerAcceptableError = lowerError * percentError;
        return upperError < upperAcceptableError && lowerError < lowerAcceptableError;
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setLowerFeederSpeed(final double speed) {
        if (!lowerFeederExecutorActive.get()) {
            lowerFeederMotorMaster.set(speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime
     *            The amount of time, in seconds, to keep the motors spinning
     *            for.
     */
    public void runLowerFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (lowerFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            lowerFeederMotorMaster.set(speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        lowerFeederMotorMaster.set(0);
                    } finally {
                        lowerFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setUpperFeederSpeed(final double speed) {
        if (!upperFeederExecutorActive.get()) {
            upperFeederMotorMaster.set(speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime
     *            The amount of time, in seconds, to keep the motors spinning
     *            for.
     */
    public void runUpperFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (upperFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotorMaster.set(speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        upperFeederMotorMaster.set(0);
                    } finally {
                        upperFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime
     *            The amount of time, in seconds, to keep the motors spinning
     *            for.
     */
    public void pulseUpperFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (upperFeederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotorMaster.set(speed);
                            Timer.delay(.02);
                        }
                        endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotorMaster.set(0);
                            Timer.delay(.02);
                        }
                        upperFeederMotorMaster.set(0);
                    } finally {
                        upperFeederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    public void setShooterTiltPosition(final double position) {
        shooterTiltPIDController.enablePID();
        shooterTiltPIDController.updateSensorLockValueWithoutReset(position);
        shooterTiltMotor.set(shooterTiltPIDController.getPIDValue());
    }

    public void setShooterTiltSpeed(final double speed) {
        shooterTiltPIDController.disablePID();
        shooterTiltMotor.set(speed);
    }

    public double getShooterTilt() {
        return shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() * 360;
    }

    @Override
    public double getAsDouble() {
        return HelperFunctions.Normalize_Gryo_Value(getShooterTilt() - SHOOTER_ENCODER_ZERO_POSITION);
    }

    public double test() {
        return shooterTiltPIDController.getSensorValue();
    }

    public double testLock() {
        return shooterTiltPIDController.getSensorLockValue();
    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        lowerShooterTargetRPM = 0;
        upperShooterTargetRPM = 0;
        lowerShooterMotor.disable();
        upperShooterMotor.disable();
        lowerFeederMotorMaster.disable();
        upperFeederMotorMaster.disable();
        shooterTiltPIDController.disablePID();
        // pickupMotor.set(ControlMode.Disabled, 0);
    }

}