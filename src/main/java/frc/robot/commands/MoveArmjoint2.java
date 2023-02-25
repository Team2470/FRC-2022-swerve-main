// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Armjoint2V2;
import frc.robot.subsystems.ProfiledArmjoint;

public class MoveArmjoint2 extends CommandBase {
  private final Armjoint2V2 m_Armjoint;
  private final double m_angle;
  /** Creates a new MoveArmjoint2. */
  public MoveArmjoint2(Armjoint2V2 armjoint, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Armjoint= armjoint;
    m_angle= angle;
    addRequirements(m_Armjoint);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Armjoint.setSetpoint(m_angle);
    m_Armjoint.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Joint Angle Command Setpoint", m_angle);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Armjoint.disable();
    m_Armjoint.stop();
  }

  
  public Rotation2d getError() {
    return m_Armjoint.getAngleFromGround().minus(Rotation2d.fromDegrees(m_angle));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double threshold = 2;
    double error = Math.abs(getError().getDegrees());
    SmartDashboard.putNumber("MoveArmJoint2 error", error);
    return error <= threshold;
  }
}
