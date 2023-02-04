// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ProfiledArmjoint;

public class MoveArmjoint2 extends CommandBase {
  private final ProfiledArmjoint m_Armjoint;
  private final double m_angle;
  /** Creates a new MoveArmjoint2. */
  public MoveArmjoint2(ProfiledArmjoint armjoint, double  angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Armjoint= armjoint;
    m_angle= angle;
    addRequirements(m_Armjoint);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Armjoint.setGoal(m_angle);
    m_Armjoint.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Armjoint.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
