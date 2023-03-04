// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.subsystems.Armjoint2V2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristJointV2;



public class GripperCloseAndWristUp extends ParallelCommandGroup {
  /** Creates a new GripperCloseAndWristUp. */

  public GripperCloseAndWristUp(ArmJoint1 m_armJoint1, Armjoint2V2 m_Armjoint2,GripperSubsystem m_Gripper, WristJointV2 m_Wrist, Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SequentialCommandGroup(
        new WaitUntilCommand(()->(m_armJoint1.getAngle().getDegrees() < 60 && m_Armjoint2.getAngleFromGround().getDegrees() < 50)),
					new RunCommand(()->m_drivetrain.setSlowMode(false))
				),
      new RunCommand(()->m_Gripper.closeGripper(),m_Gripper),
  			new SequentialCommandGroup(
          new WaitCommand(0.5),
          new ConditionalCommand( 
            new ScheduleCommand(new MoveWristJoint2(m_Wrist, -95)),
            new PrintCommand("no game piece not closing grabber"),
            m_Gripper::isGamePieceDetected)
        )
    );
  }

  // Called when the command is initially scheduled.
 
}
