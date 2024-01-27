package frc.team3171.drive;

/**
 * @author Mark Ebert
 */
public class SwerveUnitConfig {

    public enum MOTOR_TYPE {
        CTRE, REV
    }

    public enum ENCODER_TYPE {
        CTRE, REV
    }

    private final MOTOR_TYPE DRIVE_MOTOR_TYPE, SLEW_MOTOR_TYPE;
    private final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE;
    private final int SLEW_MOTOR_CAN_ID, DRIVE_MOTOR_CAN_ID, ABSOLUTE_ENCODER_CAN_ID;
    private final boolean SLEW_MOTOR_INVERTED, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_INVERTED, LOG_PID_DATA;
    private final String CANBUS;

    /**
     * Constructor
     * 
     * @param DRIVE_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param SLEW_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_INVERTED
     *            Whether or not the direction of the slew motor needs to be inverted.
     * 
     * @param ABSOLUTE_ENCODER_TYPE
     *            The encoder type used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_CAN_ID
     *            The CAN ID of the encoder used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_INVERTED
     *            Whether or not the direction of the absolute encoder needs to be inverted.
     * 
     * @param CANBUS
     *            The {@link String} of the CAN bus that the devices are located on. All three devices of the unit should be
     *            on the same bus.
     * 
     * @param LOG_PID_DATA
     *            Whether or not to log the PID data from the swerve unit.
     */
    private SwerveUnitConfig(SwerveUnitConfigBuilder build) {
        this.DRIVE_MOTOR_TYPE = build.DRIVE_MOTOR_TYPE;
        this.DRIVE_MOTOR_CAN_ID = build.DRIVE_MOTOR_CAN_ID;
        this.DRIVE_MOTOR_INVERTED = build.DRIVE_MOTOR_INVERTED;

        this.SLEW_MOTOR_TYPE = build.SLEW_MOTOR_TYPE;
        this.SLEW_MOTOR_CAN_ID = build.SLEW_MOTOR_CAN_ID;
        this.SLEW_MOTOR_INVERTED = build.SLEW_MOTOR_INVERTED;

        this.ABSOLUTE_ENCODER_TYPE = build.ABSOLUTE_ENCODER_TYPE;
        this.ABSOLUTE_ENCODER_CAN_ID = build.ABSOLUTE_ENCODER_CAN_ID;
        this.ABSOLUTE_ENCODER_INVERTED = build.ABSOLUTE_ENCODER_INVERTED;

        this.CANBUS = build.CANBUS;

        this.LOG_PID_DATA = build.LOG_PID_DATA;
    }

    public static class SwerveUnitConfigBuilder {

        // Required
        private MOTOR_TYPE DRIVE_MOTOR_TYPE, SLEW_MOTOR_TYPE;
        private int SLEW_MOTOR_CAN_ID, DRIVE_MOTOR_CAN_ID;
        private ENCODER_TYPE ABSOLUTE_ENCODER_TYPE;

        // Optional
        private boolean SLEW_MOTOR_INVERTED, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_INVERTED;
        private int ABSOLUTE_ENCODER_CAN_ID;
        private String CANBUS;
        private boolean LOG_PID_DATA;

        /**
         * Constructor
         * 
         * @param SLEW_MOTOR_TYPE
         *            The motor type of the {@link MotorController} used to control the slew motor.
         * @param SLEW_MOTOR_CAN_ID
         *            The CAN ID of the {@link MotorController} used to control the slew motor.
         * 
         * @param DRIVE_MOTOR_TYPE
         *            The motor type of the {@link MotorController} used to control the drive motor.
         * @param DRIVE_MOTOR_CAN_ID
         *            The CAN ID of the {@link MotorController} used to control the drive motor.
         * 
         * @param ABSOLUTE_ENCODER_TYPE
         *            The encoder type used to tell the positiion of the wheel.
         */
        public SwerveUnitConfigBuilder(final MOTOR_TYPE DRIVE_MOTOR_TYPE, final int DRIVE_MOTOR_CAN_ID, final MOTOR_TYPE SLEW_MOTOR_TYPE,
                final int SLEW_MOTOR_CAN_ID, final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE) {
            this.DRIVE_MOTOR_TYPE = DRIVE_MOTOR_TYPE;
            this.DRIVE_MOTOR_CAN_ID = DRIVE_MOTOR_CAN_ID;

            this.SLEW_MOTOR_TYPE = SLEW_MOTOR_TYPE;
            this.SLEW_MOTOR_CAN_ID = SLEW_MOTOR_CAN_ID;

            this.ABSOLUTE_ENCODER_TYPE = ABSOLUTE_ENCODER_TYPE;
        }

        /**
         *
         * @param driveMotorInverted
         *            Whether or not the direction of the drive motor needs to be inverted.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setDriveMotorInverted(final boolean driveMotorInverted) {
            this.DRIVE_MOTOR_INVERTED = driveMotorInverted;
            return this;
        }

        /**
         * 
         * @param slewMotorInverted
         *            Whether or not the direction of the slew motor needs to be inverted.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setSlewMotorInverted(final boolean slewMotorInverted) {
            this.SLEW_MOTOR_INVERTED = slewMotorInverted;
            return this;
        }

        /**
         * 
         * @param absoluteEncoderInverted
         *            Whether or not the direction of the absolute encoder needs to be inverted.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setAbsoluteEncoderInverted(final boolean absoluteEncoderInverted) {
            this.ABSOLUTE_ENCODER_INVERTED = absoluteEncoderInverted;
            return this;
        }

        /**
         * 
         * @param absoluteEncoderCANID
         *            The CAN ID of the encoder used to tell the positiion of the wheel.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setAbsoluteEncoderCANID(final int absoluteEncoderCANID) {
            this.ABSOLUTE_ENCODER_CAN_ID = absoluteEncoderCANID;
            return this;
        }

        /**
         * 
         * @param canbus
         *            The {@link String} of the CAN bus that the devices are located on. All three devices of the unit should be
         *            on the same bus.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setCANBUS(final String canbus) {
            this.CANBUS = canbus;
            return this;
        }

        /**
         * 
         * @param logPIDData
         *            Whether or not to log the PID data from the swerve unit.
         * @return The same {@link SwerveUnitConfigBuilder} with the specified parameter updated.
         */
        public SwerveUnitConfigBuilder setLogPIDData(final boolean logPIDData) {
            this.LOG_PID_DATA = logPIDData;
            return this;
        }

        /**
         * 
         * @return A new instance of {@link SwerveUnitConfig} with the values set to the specified values provided by the
         *         builder.
         */
        public SwerveUnitConfig build() {
            return new SwerveUnitConfig(this);
        }

    }

    public MOTOR_TYPE getDriveMotorType() {
        return DRIVE_MOTOR_TYPE;
    }

    public MOTOR_TYPE getSlewMotorType() {
        return SLEW_MOTOR_TYPE;
    }

    public ENCODER_TYPE getAbsoluteEncoderType() {
        return ABSOLUTE_ENCODER_TYPE;
    }

    public int getSlewMotorCANID() {
        return SLEW_MOTOR_CAN_ID;
    }

    public int getDriveMotorCANID() {
        return DRIVE_MOTOR_CAN_ID;
    }

    public int getAbsoluteEncoderCANID() {
        return ABSOLUTE_ENCODER_CAN_ID;
    }

    public boolean isSlewMotorInverted() {
        return SLEW_MOTOR_INVERTED;
    }

    public boolean isDriveMotorInverted() {
        return DRIVE_MOTOR_INVERTED;
    }

    public boolean isAbsoluteEncoderInverted() {
        return ABSOLUTE_ENCODER_INVERTED;
    }

    public String getCANBUS() {
        return CANBUS;
    }

    public boolean isLogPIDData() {
        return LOG_PID_DATA;
    }

}
