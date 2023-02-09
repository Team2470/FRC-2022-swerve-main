// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmJoint1 extends SubsystemBase {

  private final Solenoid m_ratchetSolenoid;
  private final TalonFX m_motor;
  private final CANCoder m_encoder;

  /** Creates a new ArmJoint.*/
  public ArmJoint1() {
    m_ratchetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmJoint1.kSolenoidChannelRatchet);
    m_motor = new TalonFX(Constants.ArmJoint1.kMotorID, Constants.ArmJoint1.kMotorCANBus.bus_name);
    m_motor.configFactoryDefault();
    m_motor.setInverted(true);
    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configReverseSoftLimitThreshold(Constants.ArmJoint1.kReverseSoftLimit);
    m_motor.configForwardSoftLimitThreshold(Constants.ArmJoint1.kForwardSoftLimit);
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.configVoltageCompSaturation(10);
    m_motor.enableVoltageCompensation(true);


    m_encoder = new CANCoder(Constants.ArmJoint1.kEncoderID, Constants.ArmJoint1.kEncoderCANBus.bus_name);
    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(true);
    m_encoder.configMagnetOffset(88.59375 - 90);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    m_motor.configRemoteFeedbackFilter(m_encoder, 0);
    m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_motor.setSensorPhase(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Joint Angle", getAngle().getDegrees());
  }

   
  public void engageRatchet(boolean on){
    m_ratchetSolenoid.set(!on);
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_encoder.getPosition());
  }
  

  public void outwards(){
    engageRatchet(false);
    m_motor.set(ControlMode.PercentOutput, .5);
  }

  public void inwards() {
    engageRatchet(true);
m_motor.set(ControlMode.PercentOutput, -.5);
  }

  public void stop() {
    engageRatchet(true);
    m_motor.neutralOutput();
  }
}
