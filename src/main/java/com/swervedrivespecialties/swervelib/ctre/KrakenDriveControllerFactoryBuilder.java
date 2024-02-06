package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;


public class KrakenDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public KrakenDriveControllerFactoryBuilder withVoltageCompensation(double minimalVoltage) {
        this.nominalVoltage = minimalVoltage;
        return this;
    }
    public KrakenDriveControllerFactoryBuilder withCurrentLimit(double limit) {
        this.currentLimit = limit;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override public ControllerImplementation create(Integer driveConfiguration, String canbus, MechanicalConfiguration mechConfiguration) {
            TalonFXConfiguration motorConfig = new TalonFXConfiguration();
            motorConfig.MotorOutput.Inverted = mechConfiguration.isDriveInverted() ? 
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            double sensorPositionCoefficient = Math.PI * mechConfiguration.getWheelDiameter() * mechConfiguration.getDriveReduction();

            if (hasCurrentLimit()) {
                motorConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            }

            if (hasVoltageCompensation()) {
                motorConfig.Voltage.PeakForwardVoltage = nominalVoltage;
                motorConfig.Voltage.PeakReverseVoltage = -nominalVoltage;
            }

            TalonFX motor = new TalonFX(driveConfiguration);

            motor.setNeutralMode(NeutralModeValue.Brake);

            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfig), "Failed to configure TalonFX");
            return new ControllerImplementation(motor, sensorPositionCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private TalonFX motor;
        private final double sensorPositionCoefficient;

        public ControllerImplementation(TalonFX motor, double sensorPositionCoefficient) {
            this.sensorPositionCoefficient = sensorPositionCoefficient;
            this.motor = motor;
        }

        @Override public Object getDriveMotor() {
            return this.motor;
        }

        @Override public void setReferenceVoltage(double voltage) {
            this.motor.setVoltage(voltage);
        }

        @Override public double getStateVelocity() {
            return this.motor.getRotorVelocity().getValue() * sensorPositionCoefficient;
        }

        @Override public double getStateDistance() {
            return this.motor.getRotorPosition().getValue() * sensorPositionCoefficient;
        }
    }
}