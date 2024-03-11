package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import static frc.team3171.HelperFunctions.Get_Displacement;
import static frc.team3171.HelperFunctions.Normalize_Value;

/**
 * @author Mark Ebert
 */
public class ThreadedPIDController {

    /**
     * Objects
     */
    private static final ScheduledExecutorService executor = Executors.newScheduledThreadPool(4);
    private final Supplier<Double> sensor;
    private final ReentrantLock START_LOCK, PID_LOCK;
    private final AtomicBoolean started, disablePID, continuous;
    private final AtomicReference<Double> kP, kI, kD, PID_MIN, PID_MAX;
    private final AtomicReference<Double> pidValue, sensorValue, sensorLockValue, minSensorValue, maxSensorValue;

    /**
     * Variables
     */
    private double sum = 0, rate = 0;
    private double proportionalTemp, sumTemp, rateTemp;
    private double lastTime = 0;

    /**
     * Constructor
     *
     * @param sensor     The sensor that the {@code PIDController} will get its sensor values from. This object must implement the {@code Sensor} Interface. Once set, this {@code DoubleSupplier}
     *                   cannot be changed during this instance.
     * @param kP         The proportional value constant for the {@code PIDController}. This value is arbitrary and based upon testing of the {@code PIDController} to find the values that works best
     *                   for your system. Once set, this values cannot be changed during this instance.
     * @param kI         The Integral value constant for the {@code PIDController}. This value is arbitrary and based upon testing of the {@code PIDController} to find the values that works best for
     *                   your system. Once set, this value cannot be changed during this instance.
     * @param kD         The Derivative value constant for the {@code PIDController}. This value is arbitrary and based upon testing of the {@code PIDController} to find the values that works best for
     *                   your system. Once set, this value cannot be changed during this instance.
     * @param PID_MIN    The minimum value that the {@code PIDController} is allowed obtain. If the value becomes smaller then the given constant, then the {@code PIDController} will also prevent
     *                   accumulation of the sum and will set the PID value to the minimum value. Once set, this value cannot be changed during this instance.
     * @param PID_MAX    The maximum value that the {@code PIDController} is allowed obtain. If the value becomes larger then the given constant, then the {@code PIDController} will also prevent
     *                   accumulation of the sum and will set the PID value to the maximum value. Once set, this value cannot be changed during this instance.
     * @param continuous
     */
    public ThreadedPIDController(Supplier<Double> sensor, double kP, double kI, double kD, double PID_MIN, double PID_MAX, boolean continuous) {
        this.sensor = sensor;
        this.START_LOCK = new ReentrantLock();
        this.PID_LOCK = new ReentrantLock();
        this.started = new AtomicBoolean();
        this.disablePID = new AtomicBoolean(true);
        this.continuous = new AtomicBoolean(continuous);

        this.kP = new AtomicReference<>(kP);
        this.kI = new AtomicReference<>(kI);
        this.kD = new AtomicReference<>(kD);
        this.PID_MIN = new AtomicReference<>(PID_MIN);
        this.PID_MAX = new AtomicReference<>(PID_MAX);

        this.pidValue = new AtomicReference<>(Double.valueOf(0));
        this.sensorValue = new AtomicReference<>(Double.valueOf(0));
        this.sensorLockValue = new AtomicReference<>(Double.valueOf(0));
        this.minSensorValue = new AtomicReference<>(Double.valueOf(0));
        this.maxSensorValue = new AtomicReference<>(Double.valueOf(0));
    }

    /**
     * Starts the PID Controller. Can only be called once.
     *
     * @param updateRate  The update rate, in milliseconds of the pid controller.
     * @param defaultZero Whether or not when the {@link ThreadedPIDController} is disabled if the sensor lock value should be set to 0 or to the current sensor value.
     * @param logData     A queue to store the generated values from the controller to graph or record.
     */
    public void start(final int updateRate, final boolean defaultZero, final ConcurrentLinkedQueue<String> logData) {
        try {
            START_LOCK.lock();
            final double startTime = Timer.getFPGATimestamp();
            if (started.compareAndSet(false, true)) {
                executor.scheduleAtFixedRate(() -> {
                    sensorValue.set(sensor.get());
                    if (disablePID.get()) {
                        if (defaultZero) {
                            updateSensorLockValue(0);
                        } else {
                            updateSensorLockValue();
                        }
                        if (logData != null) {
                            logData.add(String.format("%.4f,%.2f,%.2f,0,0,0,0", (Timer.getFPGATimestamp() - startTime), sensorValue, sensorLockValue));
                        }
                    } else {
                        try {
                            PID_LOCK.lock();
                            if (continuous.get()) {
                                pidValue.set(calculatePID(sensorLockValue.get() - sensorValue.get()));
                            } else {
                                pidValue.set(calculatePID(Get_Displacement(sensorValue.get(), sensorLockValue.get(), minSensorValue.get(), maxSensorValue.get())));
                            }
                        } finally {
                            PID_LOCK.unlock();
                        }
                        if (logData != null) {
                            logData.add(String.format("%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", (Timer.getFPGATimestamp() - startTime), getSensorValue(), getSensorLockValue(), getPIDValue(),
                                    proportionalTemp, sumTemp, rateTemp));
                        }
                    }
                }, 0L, updateRate, TimeUnit.MILLISECONDS);
            }
        } finally {
            START_LOCK.unlock();
        }
    }

    /**
     * Starts the PID Controller. Can only be called once.
     *
     * @param defaultZero
     */
    public void start(final boolean defaultZero) {
        start(20, defaultZero, null);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start(final ConcurrentLinkedQueue<String> logData) {
        start(20, false, logData);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start(final int updateRate) {
        start(updateRate, false, null);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start() {
        start(20, false, null);
    }

    /**
     * Calculates the PID Value
     *
     * @return The PID Value
     */
    private final double calculatePID(final double displacement) {
        double pid = 0, sum = this.sum;
        double kP = this.kP.get(), kI = this.kI.get(), kD = this.kD.get();
        double currentTime = Timer.getFPGATimestamp();
        rate = (displacement / ((currentTime - lastTime) * 1000));
        sum += rate;
        proportionalTemp = (kP * displacement);
        sumTemp = (kI * sum);
        rateTemp = (kD * rate);
        pid = proportionalTemp + sumTemp + rateTemp;
        lastTime = currentTime;
        // Don't let the PID value increase past PID_MAX or below PID_MIN and
        // prevent accumulation of the sum
        if (pid >= PID_MAX.get()) {
            pid = PID_MAX.get();
        } else if (pid <= PID_MIN.get()) {
            pid = PID_MIN.get();
        } else {
            this.sum = sum;
        }
        return pid;
    }

    /**
     * Forces the PID Controller to set its current desired lock value to its current sensor value.
     */
    public void updateSensorLockValue() {
        updateSensorLockValue(sensorValue.get());
    }

    /**
     * Forces the PID Controller to set its current desired lock value to the given value but doesn't reset any accumulated values.
     *
     * @param sensorLockValue The desired sensor lock value.
     */
    public void updateSensorLockValueWithoutReset(final double sensorLockValue) {
        if (continuous.get()) {
            this.sensorLockValue.set(sensorLockValue);
        } else {
            this.sensorLockValue.set(Normalize_Value(sensorLockValue, minSensorValue.get(), maxSensorValue.get()));
        }
    }

    /**
     * Forces the PID Controller to set its current desired lock value to the given value.
     *
     * @param sensorLockValue The desired sensor lock value.
     */
    public void updateSensorLockValue(final double sensorLockValue) {
        try {
            PID_LOCK.lock();
            if (continuous.get()) {
                this.sensorLockValue.set(sensorLockValue);
            } else {
                this.sensorLockValue.set(Normalize_Value(sensorLockValue, minSensorValue.get(), maxSensorValue.get()));
            }
            this.pidValue.set(Double.valueOf(0));
            this.sum = 0;
            this.rate = 0;
            this.lastTime = Timer.getFPGATimestamp();
        } finally {
            PID_LOCK.unlock();
        }
    }

    /**
     * Returns the PID Value
     *
     * @return The current PID Value
     */
    public double getPIDValue() {
        return pidValue.get();
    }

    /**
     * Returns the current sensor value from the sensor associated with this {@code PIDController}.
     *
     * @return The current sensor reading from {@code Sensor}.
     */
    public double getSensorValue() {
        return sensorValue.get();
    }

    /**
     * Returns the current sensor lock value from this {@code PIDController}.
     *
     * @return The current sensor lock value.
     */
    public double getSensorLockValue() {
        return sensorLockValue.get();
    }

    /**
     * Disables the {@code PIDController} such that the the PID value is zero and constantly resets any accumulated values and sets the lock value to the current sensor value.
     */
    public void disablePID() {
        disablePID.compareAndSet(false, true);
    }

    /**
     * Re-enables the {@code PIDController} such that it starts calculating the PID value again.
     */
    public void enablePID() {
        disablePID.compareAndSet(true, false);
    }

    /**
     * Returns if the {@code PIDController} is disabled.
     * 
     * @return
     */
    public boolean isDisabled() {
        return disablePID.get();
    }

    /**
     * Returns if the {@code PIDController} is enabled.
     */
    public boolean isEnabled() {
        return !isDisabled();
    }

    /**
     * Sets that the sensor used in this {@code PIDController} is not continuous.
     */
    public void disableContinuous() {
        continuous.compareAndSet(true, false);
    }

    /**
     * Sets that the sensor used in this {@code PIDController} is continuous.
     */
    public void enableContinuous() {
        continuous.compareAndSet(false, true);
    }

    /**
     * @return the kP
     */
    public double getkP() {
        return kP.get();
    }

    /**
     * @param kP the kP to set
     */
    public void setkP(double kP) {
        this.kP.set(kP);
    }

    /**
     * @return the kI
     */
    public double getkI() {
        return kI.get();
    }

    /**
     * @param kI the kI to set
     */
    public void setkI(double kI) {
        this.kI.set(kI);
    }

    /**
     * @return the kD
     */
    public double getkD() {
        return kD.get();
    }

    /**
     * @param kD the kD to set
     */
    public void setkD(double kD) {
        this.kD.set(kD);
    }

    /**
     * @param minValue the minValue to set
     */
    public void setMinSensorValue(double minValue) {
        this.minSensorValue.set(minValue);
    }

    /**
     * @param maxValue the maxValue to set
     */
    public void setMaxSensorValue(double maxValue) {
        this.maxSensorValue.set(maxValue);
    }

}
