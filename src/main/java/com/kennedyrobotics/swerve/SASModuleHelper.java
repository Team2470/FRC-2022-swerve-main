package com.kennedyrobotics.swerve;

import com.kennedyrobotics.swerve.ctre.TalonSRXSteerConfiguration;
import com.kennedyrobotics.swerve.ctre.TalonSRXSteerControllerFactoryBuilder;
import com.kennedyrobotics.swerve.rev.Neo550SteerConfiguration;
import com.kennedyrobotics.swerve.rev.Neo550SteerControllerFactoryBuilder;
import com.kennedyrobotics.swerve.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SASModuleHelper {

    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(SASModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
            .withVoltageCompensation(configuration.getNominalVoltage())
            .withCurrentLimit(configuration.getDriveCurrentLimit())
            .withRampRate(0.1) // This prevents the module from stuttering
            .build();
    }

    private static SteerControllerFactory<?, Neo550SteerConfiguration> getNeo550SteerFactory(SASModuleConfiguration configuration) {
        return new Neo550SteerControllerFactoryBuilder()
            .withVoltageCompensation(configuration.getNominalVoltage())
            .withPidConstants(15, 0, 0) // TODO add to SASModuleConfiguration
            .withCurrentLimit(configuration.getSteerCurrentLimit())
            .withRampRate(0.1) // This prevents the module from stuttering, TODO add to SASModuleConfiguration
            .build();
    }

    private static SteerControllerFactory<?, TalonSRXSteerConfiguration> getTalonSRXSteerFactory(SASModuleConfiguration configuration) {
        return new TalonSRXSteerControllerFactoryBuilder()
            .withVoltageCompensation(configuration.getNominalVoltage())
            .withPidConstants(25, 0, 0) // TODO add to SASModuleConfiguration
            .withCurrentLimit(configuration.getSteerCurrentLimit())
            .withRampRate(0.1) // This prevents the module from stuttering, TODO add to SASModuleConfiguration
            .build();
    }

    /**
     *
     * @param container
     * @param configuration
     * @param gearRatio
     * @param driveMotorPort
     * @param steerMotorPort
     * @param steerOffsetDegrees
     * @return
     */
    public static SwerveModule createV1(
        ShuffleboardLayout container,
        SASModuleConfiguration configuration,
        GearRatio gearRatio,
        int driveMotorPort,
        int steerMotorPort,
        Rotation2d steerOffsetDegrees
    ) {
        return new SwerveModuleFactory<> (
            gearRatio.getConfiguration(),
            getNeoDriveFactory(configuration),
            getTalonSRXSteerFactory(configuration)
        ).create(
            container,
            driveMotorPort,
            new TalonSRXSteerConfiguration(
                steerMotorPort,
                steerOffsetDegrees
            )
        );
    }

    /**
     *
     * @param container
     * @param configuration
     * @param gearRatio
     * @param driveMotorPort
     * @param steerMotorPort
     * @param steerOffsetDegrees
     * @return
     */
    public static SwerveModule createV2(
        ShuffleboardLayout container,
        SASModuleConfiguration configuration,
        GearRatio gearRatio,
        int driveMotorPort,
        int steerMotorPort,
        Rotation2d steerOffsetDegrees
    ) {
        return new SwerveModuleFactory<> (
            gearRatio.getConfiguration(),
            getNeoDriveFactory(configuration),
            getNeo550SteerFactory(configuration)
        ).create(
            container,
            driveMotorPort,
            new Neo550SteerConfiguration(
                steerMotorPort,
                steerOffsetDegrees
            )
        );
    }

    /**
     * TODO
     */
    public enum GearRatio {
        V1(SASModuleConfigurations.V1),
        V2(SASModuleConfigurations.V2);

        private final MechanicalConfiguration configuration;

        GearRatio(MechanicalConfiguration configuration) {
            this.configuration = configuration;
        }

        public MechanicalConfiguration getConfiguration() {
            return configuration;
        }

    }
}
