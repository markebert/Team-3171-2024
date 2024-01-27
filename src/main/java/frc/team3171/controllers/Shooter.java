package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

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

// Team 3171 Imports
import frc.robot.RobotProperties;

/**
 * @author Mark Ebert
 */
public class Shooter implements RobotProperties {

    // Motor Controllers
    private final CANSparkFlex lowerShooterMotor, upperShooterMotor;
    private final CANSparkMax lowerFeederMotor, upperFeederMotor, shooterTiltMotor;
    // private final SparkMaxPIDController pickupArmPIDController;

    // PID Controllers
    private final SparkPIDController lowerShooterPIDController, upperShooterPIDController, shooterTiltPIDController;
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
        lowerFeederMotor = new CANSparkMax(LOWER_FEEDER_CAN_ID, MotorType.kBrushless);
        upperFeederMotor = new CANSparkMax(UPPER_FEEDER_CAN_ID, MotorType.kBrushless);
        shooterTiltMotor = new CANSparkMax(SHOOTER_TILT_CAN_ID, MotorType.kBrushless);

        // Set if any motors need to be inverted
        lowerShooterMotor.setInverted(LOWER_SHOOTER_INVERTED);
        upperShooterMotor.setInverted(UPPER_SHOOTER_INVERTED);
        lowerFeederMotor.setInverted(LOWER_FEEDER_INVERTED);
        upperFeederMotor.setInverted(UPPER_FEEDER_INVERTED);
        shooterTiltMotor.setInverted(SHOOTER_TILT_INVERTED);

        // Configure the velocity closed loop values
        lowerShooterMotor.restoreFactoryDefaults();
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
        shooterTiltPIDController = shooterTiltMotor.getPIDController();
        shooterTiltPIDController.setP(TILT_KP);
        shooterTiltPIDController.setI(TILT_KI);
        shooterTiltPIDController.setD(TILT_KD);
        shooterTiltPIDController.setFF(TILT_KF);
        shooterTiltPIDController.setOutputRange(TILT_MIN, TILT_MAX);
        // shooterTiltPidController.setSmartMotionMaxVelocity(2000, 0);
        // shooterTiltPidController.setSmartMotionMinOutputVelocity(0, 0);
        // shooterTiltPidController.setSmartMotionMaxAccel(1500, 0);
        shooterTiltMotor.burnFlash();
        shooterTiltTargetPosition = 0;

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
            lowerFeederMotor.set(speed);
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
                            lowerFeederMotor.set(speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        lowerFeederMotor.set(0);
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
            upperFeederMotor.set(speed);
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
                            upperFeederMotor.set(speed);
                            Timer.delay(TimedRobot.kDefaultPeriod);
                        }
                        upperFeederMotor.set(0);
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
                            upperFeederMotor.set(speed);
                            Timer.delay(.02);
                        }
                        endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            upperFeederMotor.set(0);
                            Timer.delay(.02);
                        }
                        upperFeederMotor.set(0);
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
     * Sets the speed of the pickup motor to the given value.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the pickup motors to.
     */
    public void setPickupSpeed(final double speed) {
        // pickupMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setShooterTiltPosition(final double position) {
        if (position == 0) {
            shooterTiltMotor.set(0);
        } else if (this.shooterTiltTargetPosition != position) {
            shooterTiltPIDController.setReference(position, ControlType.kPosition);
        }
        this.shooterTiltTargetPosition = position;
    }

    public double getShooterTilt() {
        return shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        lowerShooterTargetRPM = 0;
        upperShooterTargetRPM = 0;
        lowerShooterMotor.disable();
        upperShooterMotor.disable();
        lowerFeederMotor.disable();
        upperFeederMotor.disable();
        // pickupMotor.set(ControlMode.Disabled, 0);
    }

}