// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class RobotTurnToAngle extends Command {

  private Drivetrain m_drivetrain;
  private PIDController m_Controller = new PIDController(.1, 0, 0);
  private double m_angle;

  /** Creates a new RobotTurnToAngle. */
  public RobotTurnToAngle(Drivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    m_Controller.enableContinuousInput(-180, 180);
    m_Controller.setTolerance(2);
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_Controller.calculate(m_drivetrain.getOdomHeading().getDegrees(), m_angle);

    SmartDashboard.putNumber("TurnToAngle Output", output);
    SmartDashboard.putNumber("TurnToAngle Error", m_Controller.getPositionError());

    m_drivetrain.drive(0, 0, output, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Controller.atSetpoint();
  }
}
