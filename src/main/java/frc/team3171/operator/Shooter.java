package frc.team3171.operator;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotProperties;
import frc.team3171.HelperFunctions;
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;
import frc.team3171.controllers.ThreadedPIDController;

/**
 * @author Mark Ebert
 */
public class Shooter implements RobotProperties {

    // Motor Controllers
    private final CANSparkFlex lowerShooterMotor, upperShooterMotor;
    private final CANSparkMax lowerFeederMotorMaster, lowerFeederMotorFollower, upperFeederMotorMaster, upperFeederMotorFollower, shooterTiltMotor;

    // Encoders
    private final RelativeEncoder lowerShooterRelativeEncoder, upperShooterRelativeEncoder;
    private final DutyCycleEncoder tiltEncoder;

    // PID Controllers
    private final ThreadedPIDController lowerShooterPIDController, upperShooterPIDController;
    private final ThreadedPIDController shooterTiltPIDController;

    // Variables
    private double lowerShooterSpeed = 0, upperShooterSpeed = 0;
    private double tiltSpeed;
    private volatile boolean shooterFlipped = false, disableShooterTilt;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean lowerFeederExecutorActive, upperFeederExecutorActive;

    /**
     * Constructor
     *
     * @throws Exception Throws a new exception if there are an invalid amount of motors in the feederCANIDArray.
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
        lowerShooterMotor.setIdleMode(IdleMode.kBrake);
        lowerShooterMotor.setInverted(LOWER_SHOOTER_INVERTED);
        lowerShooterMotor.burnFlash();

        upperShooterMotor.restoreFactoryDefaults();
        upperShooterMotor.setIdleMode(IdleMode.kBrake);
        upperShooterMotor.setInverted(UPPER_SHOOTER_INVERTED);
        upperShooterMotor.burnFlash();

        shooterTiltMotor.restoreFactoryDefaults();
        shooterTiltMotor.setIdleMode(IdleMode.kBrake);
        shooterTiltMotor.setInverted(SHOOTER_TILT_INVERTED);
        shooterTiltMotor.burnFlash();

        // Encoder Init
        lowerShooterRelativeEncoder = lowerShooterMotor.getEncoder();
        upperShooterRelativeEncoder = upperShooterMotor.getEncoder();
        tiltEncoder = new DutyCycleEncoder(SHOOTER_TILT_ID);

        lowerShooterPIDController = new ThreadedPIDController(this::getLowerShooterVelocity, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_MIN, SHOOTER_MAX, true);
        lowerShooterPIDController.start(true);

        upperShooterPIDController = new ThreadedPIDController(this::getUpperShooterVelocity, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_MIN, SHOOTER_MAX, true);
        upperShooterPIDController.start(true);

        shooterTiltPIDController = new ThreadedPIDController(this::getShooterTilt, TILT_KP, TILT_KI, TILT_KD, TILT_MIN, TILT_MAX, false);
        shooterTiltPIDController.setMinSensorValue(-180);
        shooterTiltPIDController.setMaxSensorValue(180);
        shooterTiltPIDController.start(false);

        // Initialize the executor service for concurrency
        executorService = Executors.newFixedThreadPool(2);
        executorLock = new ReentrantLock(true);

        // Initialize the AtomicBooleans to control the thread executors
        lowerFeederExecutorActive = new AtomicBoolean(false);
        upperFeederExecutorActive = new AtomicBoolean(false);

        // Background Executor
        Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(() -> {
            if (DriverStation.isDisabled()) {
                disable();
            } else {
                final double currentShooterTilt = getShooterTilt();

                // Shooter Tilt Update
                final boolean shooterTiltWithinRange = Math.abs(Get_Gyro_Displacement(getShooterTilt(), getShooterTiltSetPosition())) < SHOOTER_TILT_ALLOWED_DEVIATION;
                if (disableShooterTilt && shooterTiltWithinRange) {
                    shooterTiltMotor.disable();
                } else {
                    final double speedValue = shooterTiltPIDController.isEnabled() ? shooterTiltPIDController.getPIDValue() : tiltSpeed;
                    final boolean lowerLimit = currentShooterTilt < SHOOTER_TILT_MIN_POSITION && speedValue < 0;
                    final boolean upperLimit = currentShooterTilt > SHOOTER_TILT_MAX_POSITION && speedValue > 0;
                    shooterTiltMotor.set(lowerLimit || upperLimit ? 0 : speedValue);
                }

                // Shooter Motors Update
                // Check if the shooter is currently flipped over, if so flip the lower and
                // upper velocities
                if (currentShooterTilt < 0 && !shooterFlipped) {
                    shooterFlipped = true;
                    setShooterVelocity((int) upperShooterPIDController.getSensorLockValue(), (int) lowerShooterPIDController.getSensorLockValue(), true);
                } else if (currentShooterTilt >= 0 && shooterFlipped) {
                    shooterFlipped = false;
                    setShooterVelocity((int) upperShooterPIDController.getSensorLockValue(), (int) lowerShooterPIDController.getSensorLockValue(), true);
                }
                SmartDashboard.putBoolean("Shooter Flipped:", shooterFlipped);

                if (lowerShooterPIDController.isDisabled()) {
                    lowerShooterMotor.set(lowerShooterSpeed);
                } else {
                    lowerShooterSpeed = 0;
                    lowerShooterMotor.set(lowerShooterPIDController.getPIDValue());
                }

                if (upperShooterPIDController.isDisabled()) {
                    upperShooterMotor.set(upperShooterSpeed);
                } else {
                    upperShooterSpeed = 0;
                    upperShooterMotor.set(upperShooterPIDController.getPIDValue());
                }
            }
        }, 0, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     *
     * @param lowerShooterSpeed The speed, from -1.0 to 1.0, to set the lower shooter motor to.
     * @param upperShooterSpeed The speed, from -1.0 to 1.0, to set the upper shooter motor to.
     */
    public void setShooterSpeed(final double lowerShooterSpeed, final double upperShooterSpeed) {
        lowerShooterPIDController.disablePID();
        upperShooterPIDController.disablePID();

        this.lowerShooterSpeed = lowerShooterSpeed;
        this.upperShooterSpeed = upperShooterSpeed;
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     *
     * @param shooterSpeed The speed, from -1.0 to 1.0, to set the all of the shooter motors to.
     */
    public void setShooterSpeed(final double shooterSpeed) {
        setShooterSpeed(shooterSpeed, shooterSpeed);
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     *
     * @param lowerShooterTargetRPM The RPM to set the lower shooter motor to.
     * @param upperShooterTargetRPM The RPM to set the upper shooter motor to.
     */
    private void setShooterVelocity(final int lowerShooterTargetRPM, final int upperShooterTargetRPM, final boolean withoutReset) {
        /**
         * First check if either desired RPM is 0, if so lets the electronic brake handle the slow done rather then the PID Controller.
         */
        if (lowerShooterTargetRPM == 0) {
            lowerShooterPIDController.disablePID();
            lowerShooterSpeed = 0;
        } else {
            lowerShooterPIDController.enablePID();
            if ((int) lowerShooterPIDController.getSensorLockValue() != lowerShooterTargetRPM) {
                if (withoutReset) {
                    lowerShooterPIDController.updateSensorLockValueWithoutReset(lowerShooterTargetRPM);
                } else {
                    lowerShooterPIDController.updateSensorLockValue(lowerShooterTargetRPM);
                }
            }
        }

        if (upperShooterTargetRPM == 0) {
            upperShooterPIDController.disablePID();
            upperShooterSpeed = 0;
        } else {
            upperShooterPIDController.enablePID();
            if ((int) upperShooterPIDController.getSensorLockValue() != upperShooterTargetRPM) {
                if (withoutReset) {
                    upperShooterPIDController.updateSensorLockValueWithoutReset(upperShooterTargetRPM);
                } else {
                    upperShooterPIDController.updateSensorLockValue(upperShooterTargetRPM);
                }
            }
        }
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     *
     * @param lowerShooterTargetRPM The RPM to set the lower shooter motor to.
     * @param upperShooterTargetRPM The RPM to set the upper shooter motor to.
     */
    public void setShooterVelocity(int lowerShooterTargetRPM, int upperShooterTargetRPM) {
        if (shooterFlipped) {
            int tempRPM = lowerShooterTargetRPM;
            lowerShooterTargetRPM = upperShooterTargetRPM;
            upperShooterTargetRPM = tempRPM;
        }
        setShooterVelocity(lowerShooterTargetRPM, upperShooterTargetRPM, false);
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     *
     * @param shooterRPM The RPM to set the all of the shooter motors to.
     */
    public void setShooterVelocity(final int shooterRPM) {
        setShooterVelocity(shooterRPM, shooterRPM);
    }

    /**
     * Returns the velocity of the lower shooter motor in RPM, converted from Units per 100ms.
     *
     * @return The RPM of the lower shooter motor.
     */
    public double getLowerShooterVelocity() {
        return lowerShooterRelativeEncoder.getVelocity();
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error margin.
     *
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent error, false otherwise.
     */
    public boolean isLowerShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double error = Math.abs(lowerShooterPIDController.getSensorLockValue() - getLowerShooterVelocity());
        final double acceptableError = Math.abs(lowerShooterPIDController.getSensorLockValue() * percentError);
        return error < acceptableError;
    }

    /**
     * Returns the velocity of the upper shooter motor in RPM, converted from Units per 100ms.
     *
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterVelocity() {
        return upperShooterRelativeEncoder.getVelocity();
    }

    /**
     * Returns if the velocity of the motor is within the provided percent error margin.
     *
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent error, false otherwise.
     */
    public boolean isUpperShooterAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double error = Math.abs(upperShooterPIDController.getSensorLockValue() - getUpperShooterVelocity());
        final double acceptableError = Math.abs(upperShooterPIDController.getSensorLockValue() * percentError);
        return error < acceptableError;
    }

    /**
     * Returns the target velocity of the lower shooter motor in RPM, converted from Units per 100ms.
     *
     * @return The target RPM of the lower shooter motor.
     */
    public double getLowerShooterTargetVelocity() {
        return lowerShooterPIDController.getSensorLockValue();
    }

    /**
     * Returns the target velocity of the upper shooter motor in RPM, converted from Units per 100ms.
     *
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterTargetVelocity() {
        return upperShooterPIDController.getSensorLockValue();
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
     * Returns if the velocity of the both shooter motors is within the provided percent error margin.
     *
     * @param percentError The percent error, from 0.0 to 1.0 with 1.0 being equivilent to 100%, allowed to be considered at velocity.
     * @return true if the motors current velocity is within the given percent error, false otherwise.
     */
    public boolean isBothShootersAtVelocity(double percentError) {
        percentError = Math.abs(percentError);
        percentError = percentError > 1 ? 1.0 : percentError;
        final double upperError = Math.abs(getUpperShooterTargetVelocity() - getUpperShooterVelocity());
        final double lowerError = Math.abs(getLowerShooterTargetVelocity() - getLowerShooterVelocity());
        final double upperAcceptableError = Math.abs(upperShooterPIDController.getSensorLockValue() * percentError);
        final double lowerAcceptableError = Math.abs(lowerShooterPIDController.getSensorLockValue() * percentError);
        return upperError < upperAcceptableError && lowerError < lowerAcceptableError;
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     *
     * @param speed The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setLowerFeederSpeed(final double speed) {
        if (!lowerFeederExecutorActive.get()) {
            lowerFeederMotorMaster.set(speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running for the desired time.
     *
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning for.
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
     * @param speed The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setUpperFeederSpeed(final double speed) {
        if (!upperFeederExecutorActive.get()) {
            upperFeederMotorMaster.set(speed);
        }
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running for the desired time.
     *
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning for.
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
     * Sets the speed of the feeder motors to the given value and keeps them running for the desired time.
     *
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning for.
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
        if (!disableShooterTilt) {
            shooterTiltPIDController.enablePID();
            tiltSpeed = 0;
            if (shooterTiltPIDController.getSensorLockValue() != position) {
                shooterTiltPIDController.updateSensorLockValue(position);
            }
        }
    }

    public void setShooterTiltPosition() {
        if (!disableShooterTilt) {
            shooterTiltPIDController.enablePID();
            tiltSpeed = 0;
        }
    }

    public double getShooterTiltSetPosition() {
        return shooterTiltPIDController.getSensorLockValue();
    }

    public void setShooterTiltSpeed(final double speed) {
        if (!disableShooterTilt) {
            shooterTiltPIDController.disablePID();
            tiltSpeed = speed;
        }
    }

    public void shooterTiltEndMatch() {
        setShooterTiltPosition(getShooterTilt() > 0 ? 70 : -70);
        disableShooterTilt = true;
    }

    public void shooterTiltStartMatch() {
        disableShooterTilt = false;
    }

    public double getShooterTiltRaw() {
        return tiltEncoder.getAbsolutePosition() * 360;
    }

    public double getShooterTilt() {
        return HelperFunctions.Normalize_Gryo_Value(getShooterTiltRaw() - SHOOTER_TILT_ZERO_POSITION);
    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        tiltSpeed = 0;
        lowerShooterSpeed = 0;
        upperShooterSpeed = 0;
        shooterTiltPIDController.disablePID();
        lowerShooterPIDController.disablePID();
        upperShooterPIDController.disablePID();
        shooterTiltMotor.disable();
        lowerShooterMotor.disable();
        upperShooterMotor.disable();
        lowerFeederMotorMaster.disable();
        upperFeederMotorMaster.disable();
    }

}
