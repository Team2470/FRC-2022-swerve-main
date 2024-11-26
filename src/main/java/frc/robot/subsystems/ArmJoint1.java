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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmJoint1 extends SubsystemBase {

  private final Solenoid m_ratchetSolenoid;
  private final CANSparkMax m_motor;
  private final CANCoder m_encoder;

  /** Creates a new ArmJoint.*/
  public ArmJoint1() {
    m_ratchetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmJoint1.kSolenoidChannelRatchet);
    m_motor = new CANSparkMax(Constants.ArmJoint1.kMotorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(true);
    // m_motor.configForwardSoftLimitEnable(true);
    // m_motor.configReverseSoftLimitEnable(true);
    // m_motor.configReverseSoftLimitThreshold(Constants.ArmJoint1.kReverseSoftLimit);
    // m_motor.configForwardSoftLimitThreshold(Constants.ArmJoint1.kForwardSoftLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.enableVoltageCompensation(10);


    m_encoder = new CANCoder(Constants.ArmJoint1.kEncoderID, Constants.ArmJoint1.kEncoderCANBus.bus_name);
    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(true);
    m_encoder.configMagnetOffset(88.59375 - 90);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    //   m_motor.configRemoteFeedbackFilter(m_encoder, 0);
    //   m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    //   m_motor.setSensorPhase(true);
    
    // Reduce CAN Bus usage, since we are using this as a dumb motor for week zero we can turn down
    // a lot of the status frame periods. When we start using the encoder, then we can increase the
    // kStatus2 frame back to 20ms (or 10ms)
    //
    // See ths page for what each frame contains: https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#can-packet-structure
    //
    // Default 10ms: Applied output, Faults, Sticky Faults, Is Follower
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // Default 20ms: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    // Default 20ms: Motor Position
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    // Default 50ms: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    // Default 20ms: Alternate Encoder Velocity, Alternate Encoder Position
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    // Default 200ms: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    // Default 200ms: Duty Cycle Absolute Encoder Velocity,  Duty Cycle Absolute Encoder Frequency
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    // IDK what status 7 is, but I'm not going to touch it.

    m_motor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Joint Angle", getAngle().getDegrees());
    // SmartDashboard.putNumber("ArmJoint1 Sensor Position", m_motor.getSelectedSensorPosition());
  }

   
  public void engageRatchet(boolean on){
    m_ratchetSolenoid.set(!on);
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_encoder.getPosition());
  }
  

  public void outwards(){
    engageRatchet(false);
    if (getAngle().getDegrees() >= Constants.ArmJoint1.kForwardSoftLimit) {
      m_motor.stopMotor();
    } else {
      m_motor.set( .25);
    }
  }

  public void inwards() {
    engageRatchet(true);
    if (getAngle().getDegrees() <= Constants.ArmJoint1.kReverseSoftLimit) {
      m_motor.stopMotor();
    } else {
      m_motor.set( -.5);
    }
  }

  public void stop() {
    engageRatchet(true);
    m_motor.stopMotor();
  }

  public void resetAbsolutePosition(){
    m_encoder.setPositionToAbsolute();
  }
}
