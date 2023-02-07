// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmJoint;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //OI
  private final CommandXboxController m_controller = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final ArmJoint m_armJoint = new ArmJoint();

  private final PneumaticHub m_PneumaticHub = new PneumaticHub();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure default commands
    m_armJoint.setDefaultCommand(new RunCommand(
      () -> m_armJoint.stop(),
      m_armJoint
    ));

    // Configure the button bindings
    configureButtonBindings();

    m_PneumaticHub.enableCompressorDigital();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Configure default commands
    m_drivetrain.setDefaultCommand(new DriveWithController(m_drivetrain, m_controller.getHID()));

    new JoystickButton(m_controller.getHID(), XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(m_drivetrain::resetHeading)); // TODO this should also do something with odometry? As it freaks out
  
    new JoystickButton(m_controller.getHID(), XboxController.Button.kA.value)
     .whileTrue(new RunCommand(()->m_armJoint.upwards(),m_armJoint));

     new JoystickButton(m_controller.getHID(), XboxController.Button.kB.value)
     .whileTrue(new RunCommand(()->m_armJoint.downwards(),m_armJoint));

     m_controller.rightStick().toggleOnTrue(new RunCommand(()->{
        var latchedModuleStates = new SwerveModuleState[]{
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      };


      m_drivetrain.setModuleStates(latchedModuleStates);
     }, m_drivetrain));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
