// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PidArmCfg;

public class WristJoint extends ProfiledArmjoint {
  /** Creates a new WristJoint. */
  public WristJoint(PidArmCfg cfg, DoubleSupplier armjoint1AngleSupplier) {
    super(cfg, armjoint1AngleSupplier);

  }

  @Override
  public Rotation2d getAngleFromGround(){
    return Rotation2d.fromDegrees( getAngle().getDegrees() - m_Armjoint1AngleSupplier.getAsDouble());
  }

}
