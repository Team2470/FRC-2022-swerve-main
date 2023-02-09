// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmJoint1Outward;
import frc.robot.commands.ArmJoint2Inward;
import frc.robot.commands.ArmJoint2Outward;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveArmjoint1ToPosition;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.commands.MoveArmjoint2;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ProfiledArmjoint;
import frc.robot.subsystems.WristJoint;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  private final ArmJoint1 m_armJoint1 = new ArmJoint1();
  private final ProfiledArmjoint m_Armjoint2 = new ProfiledArmjoint(Constants.PidArmCfg.kArmjoint2, () -> m_armJoint1.getAngle().getDegrees());
  private final GripperSubsystem m_Gripper = new GripperSubsystem();
  private final PneumaticHub m_PneumaticHub = new PneumaticHub();
  private final ProfiledArmjoint m_Wrist = new WristJoint(Constants.PidArmCfg.kWrist, () -> m_Armjoint2.getAngle().getDegrees());

  //Auto
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure default commands
    m_armJoint1.setDefaultCommand(new RunCommand(
      () -> m_armJoint1.stop(),
      m_armJoint1
    ));

    m_Gripper.setDefaultCommand(new RunCommand(
      () -> m_Gripper.openGripper(),
      m_Gripper
    ));
    


    // Configure the button bindings
    configureButtonBindings();

    m_PneumaticHub.enableCompressorAnalog(90, 120);

    //Auto Selector
    m_revDigit = new RevDigit();
    m_revDigit.display("BWMP");
    m_autoSelector = new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(
      new PrintCommand("OOPS")
      ));

    //Initialize other autos here
    //to do
    
    m_autoSelector.initialize();
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
  
     m_controller.rightStick().toggleOnTrue(new RunCommand(()->{
        var latchedModuleStates = new SwerveModuleState[]{
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      };


      m_drivetrain.setModuleStates(latchedModuleStates);
     }, m_drivetrain));
    new JoystickButton(m_controller.getHID(), XboxController.Button.kA.value)
     .whileTrue(new ArmJoint1Outward(m_armJoint1));
     

     new JoystickButton(m_controller.getHID(), XboxController.Button.kB.value)
     .whileTrue(new RunCommand(()->m_armJoint1.inwards(),m_armJoint1));
     new JoystickButton(m_controller.getHID(), XboxController.Button.kX.value)
      .onTrue(new MoveArmjoint1ToPosition(m_armJoint1, Rotation2d.fromDegrees(90)));

     new JoystickButton(m_controller.getHID(), XboxController.Button.kLeftBumper.value)
     .whileTrue(new ArmJoint2Outward(m_Armjoint2));



      new JoystickButton(m_controller.getHID(), XboxController.Button.kRightBumper.value)
      .whileTrue(new ArmJoint2Inward(m_Armjoint2));

      new JoystickButton(m_controller.getHID(), XboxController.Button.kY.value)
       .onTrue(new MoveArmjoint2(m_Armjoint2, 0));

      new JoystickButton(m_controller.getHID(), XboxController.Button.kBack.value)
       .onTrue(new ArmJoint2Outward(m_Wrist));
      new JoystickButton(m_controller.getHID(), XboxController.Button.kStart.value)
       .onTrue(new ArmJoint2Inward(m_Wrist));

      new JoystickButton(m_controller.getHID(), XboxController.Button.kX.value)
      .toggleOnTrue(new RunCommand(()->m_Gripper.closeGripper(),m_Gripper));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoSelector.selected();
  }
}
