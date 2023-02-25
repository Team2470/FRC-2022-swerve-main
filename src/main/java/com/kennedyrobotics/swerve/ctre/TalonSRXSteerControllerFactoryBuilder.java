package com.kennedyrobotics.swerve.ctre;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static com.swervedrivespecialties.swervelib.ctre.CtreUtils.checkCtreError;


public class TalonSRXSteerControllerFactoryBuilder {
    private static final int kCANTimeoutMs = 250;
    private static final int kStatusFrameGeneralPeriod = 250;

    // PID configuration
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    private double rampRate = Double.NaN;

    public TalonSRXSteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public TalonSRXSteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public TalonSRXSteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public TalonSRXSteerControllerFactoryBuilder withRampRate(double rampRate) {
        this.rampRate = rampRate;
        return this;
    }

    public boolean hasRampRate() {
        return Double.isFinite(rampRate);
    }

    public <T> SteerControllerFactory<ControllerImplementation, TalonSRXSteerConfiguration> build() {
        return new FactoryImplementation<>();
    }

    public class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, TalonSRXSteerConfiguration> {


        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);

            //
            // Module specific information
            //
            container.addNumber("Raw Angle", () -> controller.getRawAngle().getDegrees());
            container.add("Steer PID", controller.pidController)
                .withWidget(BuiltInWidgets.kPIDController);
            container.addNumber("Steer PIDError", () -> Units.radiansToDegrees(controller.pidController.getPositionError()));
            container.addNumber("Steer output percent", controller.motor::getMotorOutputPercent);

        }

        @Override
        public ControllerImplementation create(TalonSRXSteerConfiguration talonSRXSteerConfiguration, String canbus, MechanicalConfiguration moduleConfiguration) {
            TalonSRXConfiguration motorConfiguration = new TalonSRXConfiguration();

            if (!hasVoltageCompensation()) {
                throw new IllegalStateException("TalonSRXSteerController voltage compensation to be set");
            }
            motorConfiguration.voltageCompSaturation = nominalVoltage;

            if (hasCurrentLimit()) {
                motorConfiguration.continuousCurrentLimit = (int) currentLimit;
                motorConfiguration.peakCurrentLimit = (int) currentLimit;
            }
            if (hasRampRate()) {
                motorConfiguration.closedloopRamp = rampRate;
            }

            motorConfiguration.neutralDeadband = 0.04;

            motorConfiguration.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
            motorConfiguration.velocityMeasurementWindow = 1;

            TalonSRX motor = new TalonSRX(talonSRXSteerConfiguration.getMotorPort());
            checkCtreError(motor.configAllSettings(motorConfiguration, kCANTimeoutMs), "Failed to configure Talon SRX settings");
            if (hasVoltageCompensation()) {
                motor.enableVoltageCompensation(true);
            }
            motor.setInverted(moduleConfiguration.isSteerInverted());
            motor.setNeutralMode(NeutralMode.Brake);


            // CAN frames
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kCANTimeoutMs), "Failed to configure Talon SRX Status 1 period");
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kCANTimeoutMs), "Failed to configure Talon SRX Status 2 period");
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kCANTimeoutMs), "Failed to configure Talon SRX Status 3 period");
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, kCANTimeoutMs), "Failed to configure Talon SRX Status 4 period");
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kCANTimeoutMs), "Failed to configure Talon SRX Status 8 period");
            checkCtreError(motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, kCANTimeoutMs), "Failed to configure Talon SRX Status 12 period");

            // Configure analog encoder
            checkCtreError(motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, kCANTimeoutMs), "Failed to set Talon SRX feedback sensor to analog");
            motor.setSensorPhase(false);

            //
            // PID Controller
            //
            if (!hasPidConstants()) {
                throw new IllegalStateException("NEO550SteerController requires PID constants");
            }


            PIDController pidController = new PIDController(
                pidProportional,
                pidIntegral,
                pidDerivative
            );

            // Limit the PID Controller's input range between -pi and pi and set the input
            // to be continuous.
            pidController.enableContinuousInput(-Math.PI, Math.PI);

            return new ControllerImplementation(motor, talonSRXSteerConfiguration.getModuleOffset(), pidController, nominalVoltage);
        }
    }

    public static class ControllerImplementation implements SteerController {

        private final TalonSRX motor;
        private final Rotation2d moduleOffset;
        private final PIDController pidController;
        private final double maxVoltage;

        private Rotation2d referenceAngle = new Rotation2d();
        private AbsoluteEncoder absoluteEncoderShim = new AbsoluteEncoder() {
            @Override
            public double getAbsoluteAngle() {
                return getAbsoluteAngle();
            }
        };

        public ControllerImplementation(TalonSRX motor, Rotation2d moduleOffset, PIDController pidController, double maxVoltage) {
            this.motor = motor;
            this.moduleOffset = moduleOffset;
            this.pidController = pidController;
            this.maxVoltage = maxVoltage;
        }


        @Override
        public Object getSteerMotor() {
            return this.motor;
        }

        @Override
        public AbsoluteEncoder getSteerEncoder() {
            return null;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngle.getRadians();
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            referenceAngle = new Rotation2d(referenceAngleRadians);

            // Calculate the steer motor output voltage from the steering PID controller.
            double steerVoltage = pidController.calculate(getStateAngle(), referenceAngle.getRadians());

            // Clamp the turn output output to between -10 and 10
            steerVoltage = MathUtil.clamp(steerVoltage, -maxVoltage, maxVoltage);

            // Convert voltage to percentage
            double steerOutput = steerVoltage / RobotController.getBatteryVoltage();

            // Set voltage
            motor.set(TalonSRXControlMode.PercentOutput, steerOutput);

        }

        /**
         * Module angle before offset is applied
         *
         * @return raw module angle
         */
        public Rotation2d getRawAngle() {
            final double nativeUnitsPerRotation = 1023;
            double nativeUnits = motor.getSelectedSensorPosition(0);

            return new Rotation2d(nativeUnits * (2.0 * Math.PI) / nativeUnitsPerRotation);
        }

        /**
         * Module angle after offset is applied
         *
         * @return module angle
         */
        @Override
        public double getStateAngle() {
            return getRawAngle().plus(moduleOffset).getRadians();
        }
    }
}