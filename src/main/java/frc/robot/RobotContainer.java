// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //OI
  private final XboxController m_controller = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final ArmJoint1 m_armJoint1 = new ArmJoint1();

  private final PneumaticHub m_PneumaticHub = new PneumaticHub();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure default commands
    m_armJoint1.setDefaultCommand(new RunCommand(
      () -> m_armJoint1.stop(),
      m_armJoint1
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
    m_drivetrain.setDefaultCommand(new DriveWithController(m_drivetrain, m_controller));

    new JoystickButton(m_controller, XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(m_drivetrain::resetHeading)); // TODO this should also do something with odometry? As it freaks out
  
    new JoystickButton(m_controller, XboxController.Button.kA.value)
     .whileTrue(new RunCommand(()->m_armJoint1.upwards(),m_armJoint1));

     new JoystickButton(m_controller, XboxController.Button.kB.value)
     .whileTrue(new RunCommand(()->m_armJoint1.downwards(),m_armJoint1));
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
