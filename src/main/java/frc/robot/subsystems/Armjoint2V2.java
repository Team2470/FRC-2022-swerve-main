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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.PidArmCfg;

public class Armjoint2V2 extends PIDSubsystem {
  /** Creates a new Armjoint2V2. */
  private final CANSparkMax m_motor;
  private final CANCoder m_encoder;
  private final PidArmCfg m_Cfg;
  private final ArmFeedforward m_feedforward;

  protected final DoubleSupplier m_Armjoint1AngleSupplier;
  public Armjoint2V2(PidArmCfg cfg, DoubleSupplier armjoint1AngleSupplier ) {
    super(
        // The PIDController used by the subsystem
        new PIDController(
          cfg.P,
          cfg.I,
          cfg.D         
        ));
        m_Cfg = cfg;

        m_Armjoint1AngleSupplier = armjoint1AngleSupplier;

        m_feedforward = new ArmFeedforward(
          m_Cfg.svolts, m_Cfg.gvolts,
          m_Cfg.vVoltSecondPerRad, m_Cfg.aVoltSecondSquaredPerRad);
        
    
        m_motor = new CANSparkMax(m_Cfg.motorID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
      //  m_motor.configForwardSoftLimitEnable(true);
      //  m_motor.configReverseSoftLimitEnable(true);
      //  m_motor.configReverseSoftLimitThreshold(m_Cfg.reverseSoftLimit);
      //  m_motor.configForwardSoftLimitThreshold(m_Cfg.forwardSoftLimit);
        m_motor.setIdleMode(IdleMode.kBrake);
        //m_motor.configVoltageCompSaturation(10);
        //m_motor.enableVoltageCompensation(true);
    
        m_encoder = new CANCoder(m_Cfg.encoderID, m_Cfg.encoderCanbus.bus_name);
        m_encoder.configFactoryDefault();
        m_encoder.configSensorDirection(m_Cfg.encoderDirection);
        m_encoder.configMagnetOffset(m_Cfg.encoderOffset);
        m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    
        // m_motor.configRemoteFeedbackFilter(m_encoder, 0);
        // m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        // m_motor.setSensorPhase(true);

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
    super.periodic();
    SmartDashboard.putNumber(m_Cfg.name + " encoderAbosoluteAngle", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber(m_Cfg.name + " encoderAngle", m_encoder.getPosition());
    SmartDashboard.putNumber(m_Cfg.name + " Motor Selected Sensor position", m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber(m_Cfg.name + " Motor Error", getController().getPositionError());
    SmartDashboard.putNumber(m_Cfg.name + " Angle From Ground", getAngleFromGround().getDegrees());
    SmartDashboard.putNumber(m_Cfg.name + " Angle From Joint", getAngle().getDegrees());
    SmartDashboard.putNumber(m_Cfg.name + " get measurement", getMeasurement());

  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_encoder.getPosition());
  }

  public Rotation2d getAngleFromGround(){
    return Rotation2d.fromDegrees(180 - m_Armjoint1AngleSupplier.getAsDouble() - getAngle().getDegrees()).unaryMinus();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here\
    return getAngleFromGround().getDegrees();
  }
  
  
  public void upwards(){
    m_motor.set(.5);
  }

  public void downwards() {
    m_motor.set(-.5);
  }
  public void stop() { 
    m_motor.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    double feedforward = m_feedforward.calculate(setpoint, 0);
    double outPutVoltage = output;

    if (outPutVoltage > m_Cfg.outPutVoltage ) {
      outPutVoltage =  m_Cfg.outPutVoltage;
    }

    m_motor.setVoltage(outPutVoltage);
    SmartDashboard.putNumber(m_Cfg.name + " Motor Output Voltage", outPutVoltage);
    SmartDashboard.putNumber(m_Cfg.name + " Motor Pid Output", output);
    SmartDashboard.putNumber(m_Cfg.name + " Motor Setpoint Position", setpoint);
    SmartDashboard.putNumber(m_Cfg.name + " Motor feedforward", feedforward);
  }

  public void resetAbsolutePosition(){
    m_encoder.setPositionToAbsolute();
  }

}
