// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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

   public static final HashMap<String, Command> eventMap = new HashMap<String, Command>() {{
      put("marker1", new PrintCommand("Passed Marker #1"));
      put("armRaise", new PrintCommand("Arm Raised to x degreese")); //: TODO: put in actual degree
   }};

   public static class Drive {
      //: Physical constants (motors, sensors, ...)
      public static final double kDriveVoltageCompensation = 10;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kWheelBaseLengthMeters = Units.inchesToMeters(25.5);
      public static final double kTrackWidthMeters = Units.inchesToMeters(20.5);

      public static final double kDriveGearReduction = SdsModuleConfigurations.MK4_L2.getDriveReduction();

      public static final double kMaxDriveVelocityMetersPerSecond = 
         DCMotor.getNEO(1).freeSpeedRadPerSec / (2) * kDriveGearReduction * kWheelDiameterMeters * (kDriveVoltageCompensation/12.0);

      public static final double kMaxAngularVelocityRadiansPerSecond = 
         kMaxDriveVelocityMetersPerSecond / Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseLengthMeters / 2.0);

      public static final SwerveDriveKinematics kDriveKinematics =
         new SwerveDriveKinematics(
            new Translation2d(kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2));
      //: IMU constants
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
            this.line = line; this.col = col;
            return this;
         }

         public ModuleConfig setSteeringID(int id) {
            this.steeringID = id; return this;
         }

         public ModuleConfig setEncoderID(int id) {
            this.encoderID = id; return this;
         }
         
         public ModuleConfig setDrivingID(int id) {
            this.drivingID = id; return this;
         }

         public ModuleConfig setOffset(double angle) {
            this.offset = Rotation2d.fromDegrees(angle); 
            return this;
         }
      }
      //: specific module config
      public static final ModuleConfig kFrontLeft = new ModuleConfig("Front Left")
         .setDrivingID(11)
         .setEncoderID(10)
         .setSteeringID(10)
         .setOffset(-112.5167456318007+180)
         .setTab(0, 0);

      public static final ModuleConfig kFrontRight = new ModuleConfig("Front Right")
         .setDrivingID(12)
         .setEncoderID(12)
         .setSteeringID(13)
         .setOffset(41.01563430557424-82-180)
         .setTab(0, 2);

      public static final ModuleConfig kBackLeft = new ModuleConfig("Back Left")
         .setDrivingID(14)
         .setEncoderID(16)
         .setSteeringID(15)
         .setOffset(-53.349609375000014)
         .setTab(0, 4);

      public static final ModuleConfig kBackRight = new ModuleConfig("Back Right")
         .setDrivingID(17)
         .setEncoderID(14)
         .setSteeringID(16)
         .setOffset(-126.03515625)
         .setTab(0, 6);
   }


   public static class ArmJoint1 {
      public static final int kSolenoidChannelRatchet = 7;
      public static final int kMotorID = 20;
      public static final int kEncoderID = 20;
      public static final CanBus kMotorCANBus = CanBus.kCanivore; 
      public static final CanBus kEncoderCANBus = CanBus.kCanivore;

      public static final int kReverseSoftLimit = 550;
      public static final int kForwardSoftLimit = 1289;
   }

   public static class Auto {
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;

      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
         new TrapezoidProfile.Constraints
            ( kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared );
   }

   public static class PidArmCfg {
      public int motorID, encoderID;
      public CanBus motorCanbus, encoderCanbus;
      public int reverseSoftLimit, forwardSoftLimit;
      public double maxVelocityRadPerSecond, maxAccelerationRadPerSecond;
      public double svolts, gvolts, vVoltSecondPerRad, aVoltSecondSquaredPerRad;
      public double P,I,D;
      public String name;
      public double encoderOffset;
      public boolean encoderDirection;
      public double outPutVoltage;

      public PidArmCfg setCanIDs (int motorID, int encoderID ) {
         this.motorID = motorID;
         this.encoderID = encoderID;
         return this;
      }

      public PidArmCfg setCanbuses(CanBus motorCanbus, CanBus encoderCanbus ) {
         this.motorCanbus = motorCanbus;
         this.encoderCanbus = encoderCanbus;
         return this;
      }

      public PidArmCfg setLimits (int reverseSoftLimit, int forwardSoftLimit) {
         this.reverseSoftLimit = reverseSoftLimit;
         this.forwardSoftLimit= forwardSoftLimit;
         return this;
      }
      
      public PidArmCfg setMotionProfileConstants (double maxVelocityRadPerSecond, double maxAccelerationRadPerSecond) {
         this.maxVelocityRadPerSecond = maxVelocityRadPerSecond;
         this.maxAccelerationRadPerSecond = maxAccelerationRadPerSecond;
         return this;
      }
      public PidArmCfg setFeedforwardConstants (double svolts, double gvolts, double vVoltSecondPerRad, double aVoltSecondSquaredPerRad) {
         this.svolts = svolts;
         this.gvolts = gvolts;
         this.vVoltSecondPerRad = vVoltSecondPerRad;
         this.aVoltSecondSquaredPerRad = aVoltSecondSquaredPerRad;
         return this;
      }
      public PidArmCfg setPID (double P, double I, double D) {
         this.P = P;
         this.I = I;
         this.D = D;
         return this;
      }

      public PidArmCfg setName(String name){
         this.name = name;
         return this;

      }

      public PidArmCfg setEncoderOffset(double encoderOffset) {
         this.encoderOffset = encoderOffset;
         return this;
      }


      public PidArmCfg setEncoderDirection(boolean direction) {
         this.encoderDirection = direction;
         return this;
      }

      public PidArmCfg setOutputVoltage(double outPutVoltage) {
         this.outPutVoltage = outPutVoltage;
         return this;
      }




      public static final PidArmCfg kArmjoint2 = new PidArmCfg()
         .setCanIDs(21, 21)
         .setCanbuses(CanBus.kCanivore, CanBus.kCanivore)
         .setLimits(0, 1960) //back  1600 min 3400
         .setMotionProfileConstants(3.14, 3.14)
         .setFeedforwardConstants(0, 0.31, 5.39, 0)
         .setPID(.3, 0, 0)
         .setName("armJoint2")
         .setEncoderOffset(-52.91015625 - 74.1796875)
         .setEncoderDirection(true)
         .setOutputVoltage(12);




      public static final PidArmCfg kWrist = new PidArmCfg()
         .setCanIDs(22, 22)
         .setCanbuses(CanBus.kCanivore, CanBus.kCanivore)
         .setLimits(-1550, 1000) //back  1600 min 3400
         .setMotionProfileConstants(3.14, 3.14)
         .setFeedforwardConstants(0, 0.1, 3.05, 0)
         .setPID(0.2, 0, 0)
         .setName("Wrist")
         .setEncoderOffset(-23.994140625 -78 + 60)
         .setEncoderDirection(false)
         .setOutputVoltage(12);
   }
   public static class Gripper {
      public static final int kSolenoidChannelGripperOpen = 1;
      public static final int kSolenoidChannelGripperClose = 0;
      public static final int kGamePieceSensorDIO = 1;
   }
}
