package frc.team3171.models;

public class ShooterShot {

    public final int lowerShooterRPM, upperShooterRPM;

    public ShooterShot(final int lowerShooterRPM, final int upperShooterRPM) {
        this.lowerShooterRPM = lowerShooterRPM;
        this.upperShooterRPM = upperShooterRPM;
    }

    public int getLowerShooterRPM() {
        return lowerShooterRPM;
    }

    public int getUpperShooterRPM() {
        return upperShooterRPM;
    }

}
