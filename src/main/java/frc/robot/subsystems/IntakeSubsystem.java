// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

  // Hardware
  private final TalonFX m_motor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {    
    m_motor = new TalonFX(10);
    m_motor.setInverted(false);
  }

  public void inward() {
    m_motor.set(ControlMode.PercentOutput, 0.5);
  }

  public void outward() {
    m_motor.set(ControlMode.PercentOutput, -0.5);
  }

  public void stop() {
    m_motor.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
