// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum CanBus {
    kRoboRIO("rio"),
    kCanivore("Canivore0");

    public final String bus_name;

    private CanBus(String value) {
      this.bus_name = value;
    }
  } 
  public static class DriveConstants {
    // : Physical constants (motors, sensors, ...)
    public static final double kDriveVoltageCompensation = 12;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelBaseLengthMeters = Units.inchesToMeters(11.375*2);
    public static final double kTrackWidthMeters = Units.inchesToMeters(12.375*2);

    public static final double kDriveGearReduction =
        SdsModuleConfigurations.MK4I_L3.getDriveReduction();

    public static final double kMaxDriveVelocityMetersPerSecond =
        DCMotor.getNEO(1).freeSpeedRadPerSec
            / (2)
            * kDriveGearReduction
            * kWheelDiameterMeters
            * (kDriveVoltageCompensation / 12.0);

    public static final double kMaxAngularVelocityRadiansPerSecond =
        kMaxDriveVelocityMetersPerSecond
            / Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseLengthMeters / 2.0);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2));
    // : IMU constants
    public static final int kPigeonID = 0;
    public static final CanBus kPigeonCANBus = CanBus.kRoboRIO;

    public static class ModuleConfig {
      public int line, col;
      public int encoderID, steeringID, drivingID;
      public String name;
      public Rotation2d offset;

      public ModuleConfig(String name) {
        this.name = name;
      }

      public ModuleConfig setTab(int line, int col) {
        this.line = line;
        this.col = col;
        return this;
      }

      public ModuleConfig setSteeringID(int id) {
        this.steeringID = id;
        return this;
      }

      public ModuleConfig setEncoderID(int id) {
        this.encoderID = id;
        return this;
      }

      public ModuleConfig setDrivingID(int id) {
        this.drivingID = id;
        return this;
      }

      public ModuleConfig setOffset(double angle) {
        this.offset = Rotation2d.fromDegrees(angle);
        return this;
      }
    }
    // : specific module config
    public static final ModuleConfig kFrontLeft =
        new ModuleConfig("Front Left")
            .setDrivingID(17)
            .setEncoderID(16)
            .setSteeringID(16)
            .setOffset(-232.5 + 180 + 90)
            .setTab(0, 0);

    public static final ModuleConfig kFrontRight =
        new ModuleConfig("Front Right")
            .setDrivingID(11)
            .setEncoderID(10)
            .setSteeringID(10)
            .setOffset(-28.1 + 90)
            .setTab(0, 2);

    public static final ModuleConfig kBackLeft =
        new ModuleConfig("Back Left")
            .setDrivingID(15)
            .setEncoderID(14)
            .setSteeringID(14)
            .setOffset(-127.3 + 90 + 180 + 90)
            .setTab(0, 4);

    public static final ModuleConfig kBackRight =
        new ModuleConfig("Back Right")
            .setDrivingID(13)
            .setEncoderID(12)
            .setSteeringID(12)
            .setOffset(-235.1 + 90 + 90)
            .setTab(0, 6);
  }

  public static class AutoConstants {
    public static final double kAutoVoltageCompensation = 10;

    public static final PIDConstants kPIDTranslation =
        new PIDConstants(5.0, 0, 0); // : PID constants for translation error
    public static final PIDConstants kPIDRotation =
        new PIDConstants(2.0, 0, 0); // : Theta rotation,
  }

  public static class VisionConstants {
    public static final Transform3d kBackLeftCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-5.89), Units.inchesToMeters(6.30), Units.inchesToMeters(9.66)),
      new Rotation3d(0,Units.degreesToRadians(-15.0),Units.degreesToRadians(150))
    );
    public static final Transform3d kBackRightCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-5.89), -Units.inchesToMeters(6.30), Units.inchesToMeters(9.66)),
      new Rotation3d(0,Units.degreesToRadians(-15.0),Units.degreesToRadians(-150))
    );
  }
}
