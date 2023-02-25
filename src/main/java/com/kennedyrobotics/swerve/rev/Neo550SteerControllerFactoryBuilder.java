package com.kennedyrobotics.swerve.rev;

import com.revrobotics.*;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;

public class Neo550SteerControllerFactoryBuilder {
    // PID configuration
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    private double rampRate = Double.NaN;

    public Neo550SteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public Neo550SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public Neo550SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public Neo550SteerControllerFactoryBuilder withRampRate(double rampRate) {
        this.rampRate = rampRate;
        return this;
    }

    public boolean hasRampRate() {
        return Double.isFinite(rampRate);
    }

    public <T> SteerControllerFactory<ControllerImplementation, Neo550SteerConfiguration> build() {
        return new FactoryImplementation<>();
    }

    public class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, Neo550SteerConfiguration> {

        public FactoryImplementation() {
        }

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
            container.addNumber("Steer output percent", controller.motor::getAppliedOutput);
        }

        @Override
        public ControllerImplementation create(Neo550SteerConfiguration steerConfiguration, String canbus, MechanicalConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(steerConfiguration.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.isSteerInverted());
            checkNeoError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
            if (!hasVoltageCompensation()) {
                throw new IllegalStateException("NEO550SteerController voltage compensation to be set");
            }
            checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");

            if (hasCurrentLimit()) {
                checkNeoError(motor.setSmartCurrentLimit((int) Math.round(currentLimit)), "Failed to set NEO current limits");
            }

            if (hasRampRate()) {
                // Configure ramp rate to help prevent stuttering
                checkNeoError(motor.setOpenLoopRampRate(rampRate), "Failed to set NEO ramp rate");
            }


            // Steer Encoder configuration
            SparkMaxAnalogSensor absoluteEncoder = motor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
            checkNeoError(absoluteEncoder.setInverted(!moduleConfiguration.isSteerInverted()), "Failed to invert analog encoder");

            // Encoder is 1 to 1 gearing with the module.
            //           1 Revolution   360 Degrees
            // Voltage * ------------ * ------------ = Voltage * 360.0/3.3
            //           3.3 Volts      1 Revolution
            checkNeoError(absoluteEncoder.setPositionConversionFactor(360.0/3.3), "Failed to set NEO encoder position conversion factor");

            // Volts    1 Revolution   360 Degrees    Voltage
            // ------ * ------------ * ------------ = ------- * 360.0/3.3
            // Second   3.3 Volts      1 Revolution   Second
            checkNeoError(absoluteEncoder.setVelocityConversionFactor(360.0/3.3), "Failed to set NEO encoder velocity conversion factor");


            // CAN status frames
            // Status 0: Applied output/Faults/Sticky Faults/Is Follower
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            // Status 1: Motor Velocity/Motor Temperature/Motor Voltage/Motor Current
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            // Status 2: Motor Position
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            // Status 3: Analog Sensor Voltage/Analog Sensor Velocity/Analog Sensor Position
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 20), "Failed to set periodic status frame 3 rate");

            //
            // PID controller
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

            return new ControllerImplementation(motor, absoluteEncoder, steerConfiguration.getModuleOffset(), pidController, nominalVoltage);
        }
    }

    /**
     *
     */
    public static class ControllerImplementation implements SteerController {

        private final CANSparkMax motor;
        private final SparkMaxAnalogSensor absoluteEncoder;
        private final Rotation2d moduleOffset;
        private final PIDController pidController;
        private final double maxVoltage;

        private Rotation2d referenceAngle = new Rotation2d();

        /**
         *
         * @param motor
         * @param absoluteEncoder
         * @param moduleOffset
         * @param pidController
         * @param maxVoltage
         */
        public ControllerImplementation(CANSparkMax motor, SparkMaxAnalogSensor absoluteEncoder, Rotation2d moduleOffset, PIDController pidController, double maxVoltage) {
            this.motor = motor;
            this.absoluteEncoder = absoluteEncoder;
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
            return new AbsoluteEncoder() {
                @Override
                public double getAbsoluteAngle() {
                    return getAbsoluteAngle();
                }

                @Override
                public Object getInternal() {
                    return absoluteEncoder;
                }
            };
        }

        /**
         *
         * @return
         */
        @Override
        public double getReferenceAngle() {
            return referenceAngle.getRadians();
        }

        /**
         *
         * @param referenceAngleRadians
         */
        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            referenceAngle = new Rotation2d(referenceAngleRadians);

            // Calculate the steer motor output voltage from the steering PID controller.
            double steerVoltage = pidController.calculate(getStateAngle(), referenceAngle.getRadians());

            // Clamp the turn output output to between -10 and 10
            steerVoltage = MathUtil.clamp(steerVoltage, -maxVoltage, maxVoltage);

            // Set voltage
            motor.setVoltage(steerVoltage);
        }

        /**
         * Module angle before offset is applied
         * @return raw module angle
         */
        public Rotation2d getRawAngle() {
            return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        }

        /**
         * Module angle after offset is applied
         * @return module angle
         */
        @Override
        public double getStateAngle() {
            return getRawAngle().plus(moduleOffset).getRadians();
        }
    }
}
