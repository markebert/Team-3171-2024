package frc.team3171.sensors;

// Java Imports
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;

// Team 3171 Imports
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

public class Pigeon2Wrapper extends Pigeon2 implements DoubleSupplier {

    public Pigeon2Wrapper(int deviceId, String canbus) {
        super(deviceId, canbus);
    }

    public Pigeon2Wrapper(int deviceId) {
        super(deviceId);
    }

    @Override
    public double getAsDouble() {
        return Normalize_Gryo_Value(getAngle());
    }

    public Supplier<Double> asSupplier() {
        return () -> {
            return getAsDouble();
        };
    }

}