// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ProfiledArmjoint;
import frc.robot.subsystems.WristJointV2;

public class WristJointInward2 extends CommandBase {
  private final WristJointV2 m_Armjoint;
  /** Creates a new ArmJoint2Inward. */
  public WristJointInward2(WristJointV2 armjoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Armjoint = armjoint;
    addRequirements(m_Armjoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Armjoint.disable();
    m_Armjoint.upwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Armjoint.disable();
    m_Armjoint.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
