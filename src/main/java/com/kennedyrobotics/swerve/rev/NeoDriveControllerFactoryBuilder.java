package com.kennedyrobotics.swerve.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;

import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;

public class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    private double rampRate = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public NeoDriveControllerFactoryBuilder withRampRate(double rampRate) {
        this.rampRate = rampRate;
        return this;
    }

    public boolean hasRampRate() {
        return Double.isFinite(rampRate);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    /**
     *
     */
    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {

        /**
         *
         * @param id
         * @param moduleConfiguration
         * @return
         */
        @Override
        public ControllerImplementation create(Integer id, String canbus, MechanicalConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.isDriveInverted());

            if (hasVoltageCompensation()) {
                // Setup voltage compensation
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }

            if (hasCurrentLimit()) {
                // Set current limit in amps
                checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
            }

            if (hasRampRate()) {
                // Configure ramp rate to help prevent stuttering
                checkNeoError(motor.setOpenLoopRampRate(rampRate), "Failed to set NEO ramp rate");
            }

            // CAN status frames
            // Status 0: Applied output/Faults/Sticky Faults/Is Follower
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            // Status 2: Motor Position
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50), "Failed to set periodic status frame 3 rate"); // TODO TUNE, not needed

            // Set neutral mode to brake
            motor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            encoder.setPositionConversionFactor(positionConversionFactor);
            encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

            return new ControllerImplementation(motor, encoder);
        }
    }

    /**
     *  TODO
     */
    private static class ControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final RelativeEncoder encoder;

        /**
         *
         * @param motor
         * @param encoder
         */
        private ControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public Object getDriveMotor() {
            return motor;
        }

        /**
         * TODO
         * @param voltage
         */
        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        /**
         * TODO
         * @return
         */
        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public double getStateDistance() {
            return encoder.getPosition();
        }
    }
}
