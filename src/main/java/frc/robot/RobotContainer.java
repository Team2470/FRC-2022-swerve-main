// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.misc.RevDigit;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BalanceOnChargeStation;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.RobotTurnToAngle;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// OI
	private final CommandXboxController m_controller = new CommandXboxController(0);
	// The robot's subsystems and commands are defined here...
	private final Drivetrain m_drivetrain = new Drivetrain();

	private final NetworkTable m_cameraTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
	private final NetworkTableEntry m_cameraSelector = m_cameraTable.getEntry("selector");
	// Auto
	private final RevDigit m_revDigit;
	private final AutoSelector m_autoSelector;


  /**
	* The container for the robot. Contains subsystems, OI devices, and commands.
	*/
  	public RobotContainer() {
	 	// CameraServer.startAutomaticCapture();

	 	// Configure default commands
	 	m_cameraSelector.setDouble(0.0);

	 	// Configure the button bindings
	 	configureButtonBindings();

	 	PathPlannerServer.startServer(5811);

		// Auto Selector
		m_revDigit = new RevDigit();
		m_revDigit.display("VERD");
		m_autoSelector = new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(
			new PrintCommand("OOPS")));

		// Initialize other autos here
		// TODO

		m_autoSelector.initialize();
		
  	}

  	private void configureButtonBindings() {
	 // Configure default commands
	 m_drivetrain.setDefaultCommand(new DriveWithController(m_drivetrain, m_controller.getHID()));

	 m_controller.start().onTrue(new InstantCommand(m_drivetrain::resetHeading)); // TODO this should also do
																											// something with odometry? As
																											// it freaks out

	 m_controller.rightStick().toggleOnTrue(new RunCommand(() -> {
		var latchedModuleStates = new SwerveModuleState[] {
			 new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
			 new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
			 new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
			 new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
		};

		m_drivetrain.setModuleStates(latchedModuleStates);
	 }, m_drivetrain));


    m_controller.povRight().whileTrue(new RobotTurnToAngle(m_drivetrain, 0));

    m_controller.povLeft().whileTrue(new RobotTurnToAngle(m_drivetrain, 180));
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
		m_drivetrain.setNominalVoltages(Constants.Auto.kAutoVoltageCompensation);
	}

	public void teleopInit() {
		m_drivetrain.setSlowMode(false);
		m_drivetrain.setNominalVoltages(Constants.Drive.kDriveVoltageCompensation);
	}

  	public Command XStop() {
	 	return new RunCommand(() -> {
			var latchedModuleStates = new SwerveModuleState[] {
				new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
				new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
			}; m_drivetrain.setModuleStates(latchedModuleStates);
		}, m_drivetrain);
  	}

	//: uses drivetrain argument
  	public Command createAutoPath(Drivetrain drivetrain, HashMap<String, Command> eventMap, String pathName,
		PathConstraints pathConstraints) {
	 		List<PathPlannerTrajectory> pathGroup = 
				PathPlanner.loadPathGroup(pathName, pathConstraints);

		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			drivetrain::getPose,
			drivetrain::resetOdometry,
			Constants.Drive.kDriveKinematics,

			new PIDConstants(5.0, 0, 0), // : PID constants for translation error
			new PIDConstants(2, 0, 0), // : Theta rotation,

			drivetrain::setModuleStates,
			eventMap, true, drivetrain
		);

		return autoBuilder.fullAuto(pathGroup);
	}
	//: uses drivetrain member 
	public Command createAutoPath(HashMap<String, Command> eventMap, String pathName, PathConstraints pathConstraints) {
		List<PathPlannerTrajectory> pathGroup = 
			PathPlanner.loadPathGroup(pathName, pathConstraints);

		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			m_drivetrain::getPose,
			m_drivetrain::resetOdometry,
			Constants.Drive.kDriveKinematics,

			new PIDConstants(5.0, 0, 0), // : PID constants for translation error
			new PIDConstants(2.0, 0, 0), // : Theta rotation,

			m_drivetrain::setModuleStates,
			eventMap, true, m_drivetrain
		);

		return autoBuilder.fullAuto(pathGroup);
	}

	public void robotPeriodic(){
	}



  	public void disabledPeriodic() {
		m_drivetrain.resetSteerEncoders();
 	}
}