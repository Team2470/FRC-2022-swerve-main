package com.kennedyrobotics.swerve;

import java.util.Objects;

/**
 * Additional Swerve and Steer module configuration
 */
public class SASModuleConfiguration {
    private double nominalVoltage = 10.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SASModuleConfiguration that = (SASModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "SASModuleConfiguration{" +
            "nominalVoltage=" + nominalVoltage +
            ", driveCurrentLimit=" + driveCurrentLimit +
            ", steerCurrentLimit=" + steerCurrentLimit +
            '}';
    }
}
