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
import frc.robot.commands.ArmJoint2Inward;
import frc.robot.commands.ArmJoint2Outward;
import frc.robot.commands.BalanceOnChargeStation;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.GripperCloseAndWristUp;
import frc.robot.commands.MoveArmsToCone2NoStradle;
import frc.robot.commands.MoveArmsToCone3NoStradle;
import frc.robot.commands.MoveArmsToCone3NoStradle2;
import frc.robot.commands.MoveArmsToCube2;
import frc.robot.commands.MoveArmsToCube3;
import frc.robot.commands.MoveArmsToCubeCone1;
import frc.robot.commands.MoveArmsToHumanPlayer;
import frc.robot.commands.MoveArmsToPickUpPosition;
import frc.robot.commands.MoveArmsToSecondConePosition;
import frc.robot.commands.MoveArmsToStartingPosition;
import frc.robot.commands.MoveWristJoint2;
import frc.robot.commands.RobotTurnToAngle;
import frc.robot.commands.WristJointInward2;
import frc.robot.commands.WristJointOutward2;
import frc.robot.subsystems.ArmJoint1;
import frc.robot.subsystems.Armjoint2V2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristJointV2;

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
	private final CommandJoystick m_buttonPad = new CommandJoystick(1);
	// The robot's subsystems and commands are defined here...
	private final Drivetrain m_drivetrain = new Drivetrain();
	private final ArmJoint1 m_armJoint1 = new ArmJoint1();
	private final Armjoint2V2 m_Armjoint2 = new Armjoint2V2(Constants.PidArmCfg.kArmjoint2,
		() -> m_armJoint1.getAngle().getDegrees());
	private final GripperSubsystem m_Gripper = new GripperSubsystem();
	private final PneumaticHub m_PneumaticHub = new PneumaticHub();
	private final WristJointV2 m_Wrist = new WristJointV2(Constants.PidArmCfg.kWrist,
		() -> m_Armjoint2.getAngleFromGround().getDegrees());

	private final NetworkTable m_cameraTable = NetworkTableInstance.getDefault().getTable("CameraPublisher");
	private final NetworkTableEntry m_cameraSelector = m_cameraTable.getEntry("selector");
	// Auto
	private final RevDigit m_revDigit;
	private final AutoSelector m_autoSelector;
	private final Vision m_vision = new Vision();


  /**
	* The container for the robot. Contains subsystems, OI devices, and commands.
	*/
  	public RobotContainer() {
	 	// CameraServer.startAutomaticCapture();

	 	// Configure default commands
	 	m_armJoint1.setDefaultCommand
			( new RunCommand(() -> m_armJoint1.stop(), m_armJoint1) );

	 	// m_Gripper.setDefaultCommand(new RunCommand(
		// () -> m_Gripper.openGripper(),
		// m_Gripper
		// ));

	 	m_cameraSelector.setDouble(0.0);

	 	// Configure the button bindings
	 	configureButtonBindings();

	 	m_PneumaticHub.enableCompressorAnalog(90, 120);
	 	PathPlannerServer.startServer(5811);

		// Auto Selector
		m_revDigit = new RevDigit();
		m_revDigit.display("VERD");
		m_autoSelector = new AutoSelector(m_revDigit, "DFLT", new SequentialCommandGroup(
			new PrintCommand("OOPS")));

		// Initialize other autos here
		m_autoSelector.registerCommand("Middle", "MID", createAutoPath(
			new HashMap<String, Command>() {{
				put("start", new SequentialCommandGroup(
					scoreLevel3Long(),
					new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)),
					new WaitCommand(1.5)
				));
				put("stop", new ScheduleCommand(Balance()));
		}}, "mid", Constants.Auto.pathConstrains));

		m_autoSelector.registerCommand("Auto31 21pts", "21-3",
			createAutoPath(new HashMap<String, Command>() {{
				put("start", scoreLevel3Long());
				put("arm-down", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
				put("stop", Balance());
		}}, "DriveDockv3", Constants.Auto.pathConstrains));

		m_autoSelector.registerCommand("TMP", "TMP", createNew2Point5Auto());
		m_autoSelector.registerCommand("2.5 point", "25GP", score2halfPointsAutoCommand());
		m_autoSelector.registerCommand("28 point auto", "CABL", score28PointsAuto());
		m_autoSelector.registerCommand("bal", "bal", Balance());


		// Legacy autos (and un tested with new arm speed)
		// m_autoSelector.registerCommand("Drop & set left", "DSL", dropAndSet());
		// m_autoSelector.registerCommand("Drop & set right", "DSR", dropAndSet());
		// m_autoSelector.registerCommand("Auto21 18pts", "2118", createAutoPath(new HashMap<String, Command>() {{
		// 	put("start", new ParallelCommandGroup(new Command[] {
		// 			new InstantCommand(() -> m_Gripper.closeGripper()),
		// 			new SequentialCommandGroup(
		// 				new WaitCommand(0.25),
		// 				new ScheduleCommand(scoreLevel3())
		// 			),
		// 			new SequentialCommandGroup(
		// 				new WaitUntilCommand(() -> m_Wrist.getAngleFromGround().getDegrees() > -5),
		// 				new RunCommand(() -> m_Gripper.openGripper(), m_Gripper).withTimeout(1),
		// 				new ScheduleCommand(
		// 					new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)
		// 				)
		// 			)
		// 	}));
		// 	put("stop", Balance());
		// }}, "DriveDockv3", Constants.Auto.pathConstrains));

		// Untested autos
		// m_autoSelector.registerCommand("MID2", "MID2", newMidAuto());
		// m_autoSelector.registerCommand("Score, Grap, Balance", "SGAB", midScoreAndBalance());

		m_autoSelector.initialize();
		
  	}

	public Command midScoreAndBalance() {
		return createAutoPath(new HashMap<String, Command>() {{
			put("start", scoreLevel3());
			put("stop", Balance());
		}}, "MiddleScorePart1", Constants.Auto.pathConstrains);
	}

	public Command dropAndSet() {
		return createAutoPath(new HashMap<String, Command>() {{ 
			put("start", scoreLevel3()); 
			put("stop", autoGetPiece());
		}}, "DSR", new PathConstraints(3, 2));
	}

	// cable side
	public Command score28PointsAuto() {
		return createAutoPath(new HashMap<String, Command>() {{
			put("start", scoreLevel3());
			put("arm-down", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			
			put("extend", new ScheduleCommand(new MoveArmsToPickUpPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("wrist-in", new ConditionalCommand(
				new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)), 
				new ScheduleCommand(new InstantCommand(() -> m_drivetrain.stop(), m_drivetrain)),
				//() -> (m_Gripper.isGamePieceDetected())
				() -> true
			));

			put("close-intake", new ScheduleCommand(new InstantCommand(() -> m_Gripper.closeGripper())));
			put("arm_up", new ScheduleCommand(new MoveArmsToCone3NoStradle(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("stop", scoreLevel3NoArmMovement());

		}}, "28-R", Constants.Auto.pathConstrains);
  	}

	public Command score2halfPointsAutoCommand() {
		return createAutoPath(new HashMap<String, Command>() {{
			put("start", scoreLevel3());
			put("arm-down", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("extend", new ScheduleCommand(new MoveArmsToPickUpPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("close-intake", new ScheduleCommand(new InstantCommand(() -> m_Gripper.closeGripper())));
			put("wrist-in", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("arm-up", new ScheduleCommand(new MoveArmsToCone3NoStradle(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("score-cube", scoreLevel3NoArmMovement());
			put("score", new ScheduleCommand(new InstantCommand(() -> m_Gripper.openGripper())));
			put("throw", new ScheduleCommand(new InstantCommand(() -> m_Gripper.openGripper())));
			put("stop", scoreLevel3());
		}}, "2.5 GP", Constants.Auto.pathConstrains);
	}

  	public Command Balance() {
		return new SequentialCommandGroup(
			new RunCommand(() -> m_drivetrain.drive(-0.5, 0, 0, false), m_drivetrain).withTimeout(0.5),
			new RunCommand(() -> m_drivetrain.drive(1.5, 0, 0, false), m_drivetrain).withTimeout(1.5),
			new BalanceOnChargeStation(m_drivetrain, 1),
			new RunCommand(() -> m_drivetrain.drive(-0.5, 0, 0, false), m_drivetrain).withTimeout(1),
			XStop()
		);
  	}

  	public Command BalanceReverse() {
		return new SequentialCommandGroup(
			new RunCommand(() -> m_drivetrain.drive(0.5, 0, 0, false), m_drivetrain).withTimeout(0.5),
			new RunCommand(() -> m_drivetrain.drive(-1.5, 0, 0, false), m_drivetrain).withTimeout(1.5),
			new BalanceOnChargeStation(m_drivetrain, -1),
			XStop()
		);
  	}

	public Command autoGetPiece() {
		return new SequentialCommandGroup(
			new MoveWristJoint2(m_Wrist, 0),
			new RunCommand(() -> m_drivetrain.drive(0.75, 0, 0, false)).until(m_Gripper::isGamePieceDetected).withTimeout(4)
		);
	}

   public Command scoreLevel3() {
	 	return new SequentialCommandGroup(
			new InstantCommand(() -> m_Gripper.closeGripper()),
			new RunCommand(() -> m_drivetrain.drive(0.02, 0, 0, false)).withTimeout(0.25),
			new RunCommand(() -> m_drivetrain.drive(0.5, 0, 0, false)).withTimeout(.3),
            new InstantCommand(()->m_drivetrain.stop()),
			new SequentialCommandGroup(
				// new WaitCommand(0.25),
				new ScheduleCommand(
					new MoveArmsToCone3NoStradle2(m_armJoint1, m_Armjoint2, m_Wrist)
				)
			), 
			new SequentialCommandGroup(
				new WaitUntilCommand(() -> {
				boolean arm1AtPickupFloor = Math
					.abs(m_armJoint1.getAngle().getDegrees() - 125) < 5;
				boolean arm2AtPickupFloor = Math
					.abs(m_Armjoint2.getAngleFromGround().getDegrees() - -39) < 5;
				boolean wristAtPickupFloor = Math
					.abs(m_Wrist.getAngleFromGround().getDegrees() - -25) < 5;
				return arm1AtPickupFloor && arm2AtPickupFloor && wristAtPickupFloor;
			}),
			new WaitCommand(0.2),
			new ScheduleCommand(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper).withTimeout(1)),
            new WaitCommand(0.5)

			// new ScheduleCommand
			// 	( new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist) )
			)
		);
  	}

	  public Command scoreLevel3Long() {
		return new SequentialCommandGroup(
		   new InstantCommand(() -> m_Gripper.closeGripper()),
		   new RunCommand(() -> m_drivetrain.drive(0.02, 0, 0, false)).withTimeout(0.25),
		   new RunCommand(() -> m_drivetrain.drive(0.5, 0, 0, false)).withTimeout(.3),
		   new InstantCommand(()->m_drivetrain.stop()),
		   new SequentialCommandGroup(
			   // new WaitCommand(0.25),
			   new ScheduleCommand(
				   new MoveArmsToCone3NoStradle2(m_armJoint1, m_Armjoint2, m_Wrist)
			   )
		   ), 
		   new SequentialCommandGroup(
			   new WaitUntilCommand(() -> {
			   boolean arm1AtPickupFloor = Math
				   .abs(m_armJoint1.getAngle().getDegrees() - 125) < 5;
			   boolean arm2AtPickupFloor = Math
				   .abs(m_Armjoint2.getAngleFromGround().getDegrees() - -39) < 5;
			   boolean wristAtPickupFloor = Math
				   .abs(m_Wrist.getAngleFromGround().getDegrees() - -25) < 5;
			   return arm1AtPickupFloor && arm2AtPickupFloor && wristAtPickupFloor;
		   }),
		   new WaitCommand(0.2),
		   new ScheduleCommand(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper).withTimeout(1)),
		   new WaitCommand(1)

		   // new ScheduleCommand
		   // 	( new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist) )
		   )
	   );
	 }

	// TODO this goes a little too fast, opened way to fast.
	public Command scoreLevel3NoArmMovement() {
		return new SequentialCommandGroup(
		   new InstantCommand(() -> m_Gripper.closeGripper()),
		   new RunCommand(() -> m_drivetrain.drive(0.02, 0, 0, false)).withTimeout(0.25),
		   new RunCommand(() -> m_drivetrain.drive(0.5, 0, 0, false)).withTimeout(.3),
		   new InstantCommand(()->m_drivetrain.stop()),
		//    new SequentialCommandGroup(
		// 	   // new WaitCommand(0.25),
		// 	   new ScheduleCommand(
		// 		   new MoveArmsToCone3NoStradle2(m_armJoint1, m_Armjoint2, m_Wrist)
		// 	   )
		//    ), 
			new WaitUntilCommand(() -> {
		 	   boolean arm1AtPickupFloor = Math
		 		   .abs(m_armJoint1.getAngle().getDegrees() - 125) < 5;
		 	   boolean arm2AtPickupFloor = Math
		 		   .abs(m_Armjoint2.getAngleFromGround().getDegrees() - -39) < 5;
		 	   boolean wristAtPickupFloor = Math
		 		   .abs(m_Wrist.getAngleFromGround().getDegrees() - -25) < 5;
		 	   return arm1AtPickupFloor && arm2AtPickupFloor && wristAtPickupFloor;
		    }),
		    new WaitCommand(0.2),
		   new ScheduleCommand(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper).withTimeout(1)),
		   new WaitCommand(0.4),

		   new ScheduleCommand( new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist) )
	   );
	 }

	public Command moveArm2ScoreLevel3() {
		return new SequentialCommandGroup(
			new InstantCommand(() -> m_Gripper.closeGripper()),
			new ScheduleCommand(
				new MoveArmsToCone3NoStradle2(m_armJoint1, m_Armjoint2, m_Wrist)
			), 
			new SequentialCommandGroup(
				new WaitUntilCommand(() -> {
				boolean arm1AtPickupFloor = Math
					.abs(m_armJoint1.getAngle().getDegrees() - 125) < 5;
				boolean arm2AtPickupFloor = Math
					.abs(m_Armjoint2.getAngleFromGround().getDegrees() - -39) < 5;
				boolean wristAtPickupFloor = Math
					.abs(m_Wrist.getAngleFromGround().getDegrees() - -25) < 5;
				return arm1AtPickupFloor && arm2AtPickupFloor && wristAtPickupFloor;
			})
			)
		);
	}
  /**
	* Use this method to define your button->command mappings. Buttons can be
	* created by
	* instantiating a {@link GenericHID} or one of its subclasses ({@link
	* edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	* it to a {@link
	* edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	*/
	private Command createNew2Point5Auto() {
		return createAutoPath(new HashMap<String, Command>() {{
			put("start", scoreLevel3());
			put("arm-down", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("extend", new ScheduleCommand(new MoveArmsToPickUpPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("close-intake", new ScheduleCommand(new InstantCommand(() -> m_Gripper.closeGripper())));
			put("wrist-in", new ScheduleCommand(new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("arm-up", new ScheduleCommand(new MoveArmsToCone3NoStradle(m_armJoint1, m_Armjoint2, m_Wrist)));
			put("score", new SequentialCommandGroup(
				scoreLevel3NoArmMovement(),
				new InstantCommand(() -> m_Gripper.openGripper())
			));
			put("stop", scoreLevel3());
		}}, "TMP", new PathConstraints(3, 2));
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

	 m_controller.y().onTrue(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper));

	 m_controller.rightBumper()
		  .onTrue(new GripperCloseAndWristUp(m_armJoint1, m_Armjoint2, m_Gripper, m_Wrist, m_drivetrain));

    m_controller.povRight().whileTrue(new RobotTurnToAngle(m_drivetrain, 0));

    m_controller.povLeft().whileTrue(new RobotTurnToAngle(m_drivetrain, 180));

	m_controller.back().onTrue(new InstantCommand(
		()->m_Gripper.enableRightSight(!m_Gripper.getRightSightEnabled())
	));

    new Trigger(() -> {
          // boolean arm1AtScoreLow = Math.abs(m_armJoint1.getAngle().getDegrees() - 50) <
          // 5;
          // boolean arm2AtScoreLow =
          // Math.abs(m_Armjoint2.getAngleFromGround().getDegrees() - 32) < 5;
          // boolean wristAtScoreLow = Math.abs(m_Wrist.getAngleFromGround().getDegrees()
          // - 0) < 5;
          // boolean armAtScoreLow = arm1AtScoreLow && arm2AtScoreLow && wristAtScoreLow;

			 boolean arm1AtPickupFloor = Math.abs(m_armJoint1.getAngle().getDegrees() - 50) < 5;
			 boolean arm2AtPickupFloor = Math.abs(m_Armjoint2.getAngleFromGround().getDegrees() - 41) < 5;
			 boolean wristAtPickupFloor = Math.abs(m_Wrist.getAngleFromGround().getDegrees() - 0) < 5;
			 boolean armAtPickupFloor = arm1AtPickupFloor && arm2AtPickupFloor && wristAtPickupFloor;

			 boolean arm1AtHP = Math.abs(m_armJoint1.getAngle().getDegrees() - 78) < 3;
			 boolean arm2AtHP =  Math.abs(m_Armjoint2.getAngleFromGround().getDegrees() - -22) < 5;
			 boolean wristAtHP = Math.abs(m_Wrist.getAngleFromGround().getDegrees() - 0) < 5;
			 boolean armAtHP = arm1AtHP && arm2AtHP && wristAtHP;

			 return m_Gripper.isGamePieceDetected() && (armAtPickupFloor || armAtHP);
		  }).onTrue(new GripperCloseAndWristUp(m_armJoint1, m_Armjoint2, m_Gripper, m_Wrist, m_drivetrain)
		  .alongWith(new StartEndCommand(
			()-> m_controller.getHID().setRumble(RumbleType.kBothRumble, .3),
			()-> m_controller.getHID().setRumble(RumbleType.kBothRumble, 0)
			).withTimeout(.2)).alongWith(new InstantCommand(()->m_vision.showGrid()))
		  );

	 // m_controller.x().onTrue(
	 // new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2,
	 // m_Wrist).beforeStarting(()->m_drivetrain.setSlowMode(false))
	 // );
	 // m_buttonPad.button(1).whileTrue(
	 // new
	 // ArmJoint1Outward(m_armJoint1).beforeStarting(()->m_drivetrain.setSlowMode(true))
	 // );
	 m_buttonPad.button(4).onTrue(
		  new MoveArmsToCube2(m_armJoint1, m_Armjoint2, m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	 // m_buttonPad.button(5).whileTrue(
	 // new RunCommand(()->m_armJoint1.inwards(),
	 // m_armJoint1).beforeStarting(()->m_drivetrain.setSlowMode(true))
	 // );
	 m_buttonPad.button(5).onTrue(
		  new MoveArmsToCubeCone1(m_armJoint1, m_Armjoint2, m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	m_buttonPad.button(8).onTrue(
			new MoveArmsToCubeCone1(m_armJoint1, m_Armjoint2, m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	 m_buttonPad.button(2).onTrue(
		  new MoveArmsToCone3NoStradle(m_armJoint1, m_Armjoint2, m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));

	 m_buttonPad.button(3).onTrue(
		  new MoveArmsToCube3(m_armJoint1, m_Armjoint2, m_Wrist).beforeStarting(() -> m_drivetrain.setSlowMode(true)));

	//  m_buttonPad.button(6).whileTrue(
		//   new ArmJoint2Inward(m_Armjoint2).beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	 m_buttonPad.button(6).onTrue(
		  new ScheduleCommand(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper)).alongWith(
				new MoveArmsToHumanPlayer(m_armJoint1, m_Armjoint2, m_Wrist)
					 .beforeStarting(() -> m_drivetrain.setSlowMode(true))).beforeStarting(()->m_vision.showGripper())
					 );

	 m_buttonPad.button(7).whileTrue(
		  new WristJointOutward2(m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	 m_buttonPad.button(11).whileTrue(
		  new WristJointInward2(m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true)));
	 m_buttonPad.button(12).onTrue(
		  new MoveWristJoint2(m_Wrist, 0).beforeStarting(() -> m_drivetrain.setSlowMode(true)));

	 m_buttonPad.button(10).onTrue(
		  new ParallelCommandGroup(
				new MoveArmsToStartingPosition(m_armJoint1, m_Armjoint2, m_Wrist).beforeStarting(()->m_vision.showGrid()),
				new SequentialCommandGroup(
					 new WaitUntilCommand(() -> (m_armJoint1.getAngle().getDegrees() < 60
						  && m_Armjoint2.getAngleFromGround().getDegrees() > -20)),
					 new RunCommand(() -> m_drivetrain.setSlowMode(false))))

	 );
	 	m_buttonPad.button(9).onTrue(
		  	new ScheduleCommand(new RunCommand(() -> m_Gripper.openGripper(), m_Gripper)).alongWith(
				new MoveArmsToPickUpPosition(m_armJoint1, m_Armjoint2, m_Wrist)
					.beforeStarting(() -> m_drivetrain.setSlowMode(true)).beforeStarting(()->m_vision.showGripper())
				)
		);
	 	m_buttonPad.button(1).onTrue(
		  new MoveArmsToCone2NoStradle(m_armJoint1, m_Armjoint2, m_Wrist)
				.beforeStarting(() -> m_drivetrain.setSlowMode(true))
		);
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
		m_Gripper.enableRightSight(true);
	}

	public void teleopInit() {
		m_drivetrain.setSlowMode(false);
		m_drivetrain.setNominalVoltages(Constants.Drive.kDriveVoltageCompensation);
		m_Gripper.enableRightSight(true);
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
		SmartDashboard.putBoolean("right sight enabled",m_Gripper.getRightSightEnabled());
	}



  	public void disabledPeriodic() {
		m_drivetrain.resetSteerEncoders();
		m_Wrist.resetAbsolutePosition();
		m_armJoint1.resetAbsolutePosition();
		m_Armjoint2.resetAbsolutePosition();
  	}
}