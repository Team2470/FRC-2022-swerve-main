// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.PidArmCfg;

public class ProfiledArmjoint extends ProfiledPIDSubsystem {
  private final WPI_TalonFX m_motor;
  private final CANCoder m_encoder;
  private final PidArmCfg m_Cfg;
  private final ArmFeedforward m_feedforward;

  private final DoubleSupplier m_Armjoint1AngleSupplier;



  /** Creates a new ProfiledArmjoint. */
  public ProfiledArmjoint(PidArmCfg cfg, DoubleSupplier armjoint1AngleSupplier) {
    
    super(

      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
          cfg.P,
          cfg.I,
          cfg.D,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(cfg.maxVelocityRadPerSecond, cfg.maxAccelerationRadPerSecond)));
    
      
    m_Cfg = cfg;

    m_Armjoint1AngleSupplier = armjoint1AngleSupplier;
    
    m_feedforward = new ArmFeedforward(
      m_Cfg.svolts, m_Cfg.gvolts,
      m_Cfg.vVoltSecondPerRad, m_Cfg.aVoltSecondSquaredPerRad);

    m_motor = new WPI_TalonFX(m_Cfg.motorID, m_Cfg.motorCanbus.bus_name);
    m_motor.configFactoryDefault();
    m_motor.setInverted(true);
    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configReverseSoftLimitThreshold(m_Cfg.reverseSoftLimit);
    m_motor.configForwardSoftLimitThreshold(m_Cfg.forwardSoftLimit);
    m_motor.setNeutralMode(NeutralMode.Brake);
    //m_motor.configVoltageCompSaturation(10);
    //m_motor.enableVoltageCompensation(true);

    m_encoder = new CANCoder(m_Cfg.encoderID, m_Cfg.encoderCanbus.bus_name);
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
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("encoderAngle", m_encoder.getPosition());
    SmartDashboard.putNumber("encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoderAngle", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Motor Stator Current", m_motor.getStatorCurrent());
    SmartDashboard.putNumber("Motor Supply Currnet", m_motor.getSupplyCurrent());
    SmartDashboard.putNumber("Motor Error", getController().getPositionError());


  }


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    
    // Use the output (and optionally the setpoint) here

    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    m_motor.setVoltage(output + feedforward);

    SmartDashboard.putNumber("Motor Output Power", output);
    SmartDashboard.putNumber("Motor Setpoint Position", setpoint.position);
    SmartDashboard.putNumber("Motor Setpoint Velocity", setpoint.velocity);
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_encoder.getPosition());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 180 - m_Armjoint1AngleSupplier.getAsDouble() - getAngle().getDegrees();
  }
  
  
  public void upwards(){
    m_motor.set(ControlMode.PercentOutput, .5);
  }

  public void downwards() {
    m_motor.set(ControlMode.PercentOutput, -.5);
  }
  public void stop() { 
    m_motor.neutralOutput();
  }
}
