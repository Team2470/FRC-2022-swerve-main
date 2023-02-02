// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmJoint1;

public class MoveArmjoint1ToPosition extends CommandBase {
  private enum OutwardState{
    kWaitForRatchet,
    KRunning
  }
  private enum Direction {
    kOutward,
    kInward
  }

  private final ArmJoint1 m_armJoint1;
  private final Rotation2d m_setpoint;
  private Direction m_direction;
  private final Timer m_timer = new Timer();
  private OutwardState m_armstate;

  /** Creates a new MoveArmjoint1ToPosition. */
  public MoveArmjoint1ToPosition(ArmJoint1 armJoint1, Rotation2d setpoint) {
    m_armJoint1 = armJoint1;
    m_setpoint = setpoint;
    addRequirements(armJoint1);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_armstate = OutwardState.kWaitForRatchet;
    m_timer.reset();
    m_timer.start();
    
    if (m_armJoint1.getAngle().minus(m_setpoint).getDegrees() > 0) {
      m_direction = Direction.kInward;
    } else {
    
      m_direction = Direction.kOutward;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_direction) {
      case kInward:
        m_armJoint1.inwards();
        break;
      case kOutward:
        m_armJoint1.engageRatchet(false);
        switch(m_armstate){
          case kWaitForRatchet:
            if (m_timer.advanceIfElapsed(0.1)){
              m_armstate = OutwardState.KRunning;
            }
            break;
          case KRunning:
            m_armJoint1.outwards();
            break;
        }
      break;
        
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armJoint1.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_direction == Direction.kInward){
      return m_armJoint1.getAngle().getDegrees() < m_setpoint.plus(Rotation2d.fromDegrees(1)).getDegrees();
    } else {
      return m_armJoint1.getAngle().getDegrees() > m_setpoint.minus(Rotation2d.fromDegrees(1)).getDegrees();
    }
  }
}
