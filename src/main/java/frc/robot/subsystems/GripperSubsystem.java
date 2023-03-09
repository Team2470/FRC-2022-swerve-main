// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

    private final DoubleSolenoid m_gripperSolenoid;
    private final DigitalInput m_gamePieceSensor;
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    m_gripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Gripper.kSolenoidChannelGripperOpen, Constants.Gripper.kSolenoidChannelGripperClose);
    m_gamePieceSensor = new DigitalInput(Constants.Gripper.kGamePieceSensorDIO);
  }

  public Boolean isGamePieceDetected(){
    return !m_gamePieceSensor.get();
  }

  public void openGripper(){
    m_gripperSolenoid.set(Value.kForward);
  }
  
  public void closeGripper(){
    m_gripperSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Gripper Game Piece Detected", isGamePieceDetected());

  }
}
