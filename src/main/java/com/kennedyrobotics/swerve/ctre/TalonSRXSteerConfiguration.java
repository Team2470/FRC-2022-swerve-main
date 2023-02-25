package com.kennedyrobotics.swerve.ctre;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Objects;

public class TalonSRXSteerConfiguration {
    private final int motorPort;
    private final Rotation2d moduleOffset;

    public TalonSRXSteerConfiguration(int motorPort, Rotation2d moduleOffset) {
        this.motorPort = motorPort;
        this.moduleOffset = moduleOffset;
    }

    public int getMotorPort() {
        return motorPort;
    }

    public Rotation2d getModuleOffset() {
        return moduleOffset;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TalonSRXSteerConfiguration that = (TalonSRXSteerConfiguration) o;
        return getMotorPort() == that.getMotorPort() && moduleOffset.equals(that.moduleOffset);
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort());
    }

    @Override
    public String toString() {
        return "TalonSRXSteerConfiguration{" +
            "motorPort=" + motorPort +
            ", moduleOffset=" + moduleOffset +
            '}';
    }
}
