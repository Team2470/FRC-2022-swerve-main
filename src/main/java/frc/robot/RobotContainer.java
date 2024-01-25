// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PivotLEDs;
import frc.robot.subsystems.PivotTemp;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PivotLEDs.Side;
import frc.robot.vision.VisionIOPhoton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final CommandXboxController m_controller = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem m_vision =
      new VisionSubsystem(
          new VisionIOPhoton("Back-Left", VisionConstants.kBackLeftCamera),
          new VisionIOPhoton("Back-Right", VisionConstants.kBackRightCamera));
  private final PivotLEDs m_pivotLEDsLeft = new PivotLEDs(Side.kLeft, m_vision.getCameras()[0]);
  private final PivotLEDs m_pivotLEDsRight = new PivotLEDs(Side.kRight, m_vision.getCameras()[1]);
  private final Drivetrain m_drivetrain = new Drivetrain(m_vision);
  private final PivotTemp m_pivotTemp = new PivotTemp();

  // Auto
  private final RevDigit m_revDigit;
  private final AutoSelector m_autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();

    // Configure default commands

    // Configure the button bindings
    configureButtonBindings();

    // PathPlannerServer.startServer(5811);`

    // Auto Selector
    m_revDigit = new RevDigit();
    m_revDigit.display("3081");
    m_autoSelector =
        new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(new PrintCommand("OOPS")));

    // Initialize other autos here
    // TODO
    m_autoSelector.registerCommand("Test", "TEST", m_drivetrain.createAutoPath(
      null, "New Auto", 
      new PathConstraints(3, 2, 
        Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond, 
        Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond)
    ));

    m_autoSelector.initialize();
  }

  private void configureButtonBindings() {
    // Configure default commands
    m_drivetrain.setDefaultCommand(
        new DriveWithController(
            m_drivetrain,
            // X Move Velocity - Forward
            () -> -m_controller.getHID().getLeftY(),

            // Y Move Velocity - Strafe
            () -> -m_controller.getHID().getLeftX(),

            // Rotate Angular velocity
            () -> {
              double leftTrigger = m_controller.getHID().getLeftTriggerAxis();
              double rightTrigger = m_controller.getHID().getRightTriggerAxis();

              if (leftTrigger < rightTrigger) {
                return -rightTrigger;
              } else {
                return leftTrigger;
              }
            },

            // Field Orientated
            () -> !m_controller.getHID().getAButton(),

            // Slow Mode
            () -> m_drivetrain.getSlowMode() || m_controller.getHID().getXButton(),

            // Disable X Movement
            () -> m_controller.getHID().getBButton(),

            // Disable Y Movement
            () -> false,

            // Heading Override
            () -> {
              switch (m_controller.getHID().getPOV()) {
                case 0:
                  return 0.0;
                case 180:
                  return 180.0;
                default:
                  return null;
              }
            }));

    m_controller
        .start()
        .onTrue(new InstantCommand(m_drivetrain::resetHeading)); // TODO this should also do
    // something with odometry? As
    // it freaks out

    m_controller.rightStick().toggleOnTrue(m_drivetrain.xStop());

    // m_controller.povRight().whileTrue(new RobotTurnToAngle(m_drivetrain, 0));

    // m_controller.povLeft().whileTrue(new RobotTurnToAngle(m_drivetrain, 180));

    // m_controller.povUp().whileTrue(m_pivotTemp.setPowerCmd(0.6));
    // m_controller.povDown().whileTrue(m_pivotTemp.setPowerCmd(-0.6));

    m_controller.back().onTrue(m_pivotTemp.resetEncoderCmd());
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

  public void autonomousInit() {
    m_drivetrain.resetHeading();
    m_drivetrain.setNominalVoltages(AutoConstants.kAutoVoltageCompensation);
  }

  public void teleopInit() {
    m_drivetrain.setSlowMode(false);
    m_drivetrain.setNominalVoltages(DriveConstants.kDriveVoltageCompensation);
  }

  public void robotPeriodic() {}

  public void disabledPeriodic() {
    m_drivetrain.resetSteerEncoders();
  }
}
