// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmJoint2;

public class ProfiledArmjoint extends ProfiledPIDSubsystem {
  private final WPI_TalonFX m_motor;
  private final CANCoder m_encoder;

  private final ArmFeedforward m_feedforward = 
    new ArmFeedforward(
        ArmJoint2.kSVolts, ArmJoint2.KGVolts,
        ArmJoint2.kVVoltSecondPerRad, ArmJoint2.kAVoltSecondSquaredPerRad);

  /** Creates a new ProfiledArmjoint. */
  public ProfiledArmjoint() {

    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
          Constants.ArmJoint2.kP,
          Constants.ArmJoint2.kI,
          Constants.ArmJoint2.kD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(Constants.ArmJoint2.kMaxVelocityRadPerSecond, Constants.ArmJoint2.kMaxAccelerationRadPerSecond)));
    
    m_motor = new WPI_TalonFX(Constants.ArmJoint2.kMotorID, Constants.ArmJoint2.kMotorCanbus.bus_name);
    m_motor.configFactoryDefault();
    m_motor.setInverted(true);
    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configReverseSoftLimitThreshold(Constants.ArmJoint2.kReverseSoftLimit);
    m_motor.configForwardSoftLimitThreshold(Constants.ArmJoint2.kForwardSoftLimit);
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.configVoltageCompSaturation(10);
    m_motor.enableVoltageCompensation(true);

    m_encoder = new CANCoder(Constants.ArmJoint2.kEncoderID, Constants.ArmJoint2.kEncoderCANBus.bus_name);
    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(true);
    m_encoder.configMagnetOffset(90);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    m_motor.configRemoteFeedbackFilter(m_encoder, 0);
    m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_motor.setSensorPhase(true);


  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    m_motor.setVoltage(output + feedforward);


  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getPosition();
  }
}
