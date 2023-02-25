// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.subsystems.Armjoint2V2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ProfiledArmjoint;
import frc.robot.subsystems.WristJoint;
import frc.robot.subsystems.WristJointV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmsToStartingPosition extends SequentialCommandGroup {

  /** Creates a new MoveArmsToStartingPosition. */
  public MoveArmsToStartingPosition(ArmJoint1 armJoint1,  Armjoint2V2 Armjoint2, WristJointV2 Wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new MoveWristJoint2(Wrist, -95),
        new SequentialCommandGroup(
          new WaitUntilCommand(()->(Wrist.getAngleFromGround().getDegrees() < 0)),
          new ParallelCommandGroup(
            new MoveArmjoint1ToPosition(armJoint1, Rotation2d.fromDegrees(50)).repeatedly(),
            new SequentialCommandGroup(
              new WaitUntilCommand(()->(armJoint1.getAngle().getDegrees() < 75)),
              new MoveArmjoint2(Armjoint2, 41)   
            )
          )
          )
      )
    );
  }
}
