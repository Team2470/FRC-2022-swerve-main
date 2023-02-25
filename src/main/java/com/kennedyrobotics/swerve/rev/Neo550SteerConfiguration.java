package com.kennedyrobotics.swerve.rev;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Objects;

public class Neo550SteerConfiguration {
    private final int motorPort;
    private final Rotation2d moduleOffset;

    public Neo550SteerConfiguration(int motorPort, Rotation2d moduleOffset) {
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
        Neo550SteerConfiguration that = (Neo550SteerConfiguration) o;
        return getMotorPort() == that.getMotorPort() && moduleOffset.equals(that.moduleOffset);
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort());
    }

    @Override
    public String toString() {
        return "Neo550SteerConfiguration{" +
            "motorPort=" + motorPort +
            ", moduleOffset=" + moduleOffset +
            '}';
    }
}
