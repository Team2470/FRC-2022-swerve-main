package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveWithController extends Command {
  private static final double kDeadband = 0.15;

  private final Drivetrain drive;

  // Inputs
  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private final DoubleSupplier angularVelocitySupplier;
  private final BooleanSupplier fieldOrientedSupplier;
  private final BooleanSupplier slowModeSupplier;
  private final BooleanSupplier disableXMovementSupplier;
  private final BooleanSupplier disableYMovementSupplier;
  private final Supplier<Double> headingOverrideSupplier;

  // State
  private boolean fieldOrient;
  private boolean lastMovingState = false;
  private SwerveModuleState[] latchedModuleStates;
  private final SlewRateLimiter xFilter = new SlewRateLimiter(3);
  private final SlewRateLimiter yFilter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotateFilter = new SlewRateLimiter(5);

  private final TrapezoidProfile.Constraints headingControllerConstraints =  new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularVelocityRadiansPerSecond/4.0, 4*Math.PI);
  private final ProfiledPIDController headingController = new ProfiledPIDController(2.0, 0, 0, headingControllerConstraints);
  private boolean lastHeadingControllerEnabled = false;

  // Keep track of the last 5 module angles
  private static final int kAngleHistoryMilliseconds = 100;
  private static final int kAngleHistoryLength = kAngleHistoryMilliseconds / 20;
  private CircularBuffer[] lastModuleAngles = {
    new CircularBuffer(kAngleHistoryLength),
    new CircularBuffer(kAngleHistoryLength),
    new CircularBuffer(kAngleHistoryLength),
    new CircularBuffer(kAngleHistoryLength)
  };

  public DriveWithController(
      Drivetrain drive,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier angularVelocitySupplier,
      BooleanSupplier fieldOrientedSupplier,
      BooleanSupplier slowModeSupplier,
      BooleanSupplier disableXMovementSupplier,
      BooleanSupplier disableYMovementSupplier,
      Supplier<Double> headingOverrideSupplier) {

    this.drive = drive;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;
    this.slowModeSupplier = slowModeSupplier;
    this.disableXMovementSupplier = disableXMovementSupplier;
    this.disableYMovementSupplier = disableYMovementSupplier;
    this.headingOverrideSupplier = headingOverrideSupplier;

    addRequirements(drive);

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    fieldOrient = true;
    lastMovingState = false;

    // Flush buffers with current module angles
    SwerveModuleState[] currentModuleState = drive.getModuleStates();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < kAngleHistoryLength; j++) {
        lastModuleAngles[i].addLast(currentModuleState[i].angle.getDegrees());
      }
    }
  }

  @Override
  public void execute() {
    // Read gamepad state
    double xMove = xVelocitySupplier.getAsDouble();
    double yMove = yVelocitySupplier.getAsDouble();
    double rotate = angularVelocitySupplier.getAsDouble();
    fieldOrient = fieldOrientedSupplier.getAsBoolean();

    if (disableXMovementSupplier.getAsBoolean()) {
      xMove = 0;
    }
    if (disableYMovementSupplier.getAsBoolean()) {
      yMove = 0;
    }

    Double headingOverride = headingOverrideSupplier.get();

    // Apply filter
    xMove = xFilter.calculate(xMove);
    yMove = yFilter.calculate(yMove);
    rotate = rotateFilter.calculate(rotate);

    // Apply deadband
    xMove = MathUtil.applyDeadband(xMove, kDeadband);
    yMove = MathUtil.applyDeadband(yMove, kDeadband);
    rotate = MathUtil.applyDeadband(rotate, kDeadband);

    // Determine if the robot should be moving,
    boolean moving = xMove != 0 || yMove != 0 || rotate != 0;

    // Capture module angles
    SwerveModuleState[] currentModuleState = drive.getModuleStates();
    for (int i = 0; i < 4; i++) {
      lastModuleAngles[i].addLast(currentModuleState[i].angle.getDegrees());
    }

    if (moving) {
      xMove = Math.copySign(xMove * xMove, xMove);
      yMove = Math.copySign(yMove * yMove, yMove);
      rotate = Math.copySign(rotate * rotate, rotate);

      if (slowModeSupplier.getAsBoolean()) {
        xMove *= 0.3;
        yMove *= 0.3;
        rotate *= 0.25;
      } else {
        xMove *= 1.0;
        yMove *= 1.0;
        rotate *= 0.5;
      }

      // Now we need to map the percentages to Meters (or Radians) per second, as that is what the
      // drive train
      // subsystem accepts
      xMove *= Constants.DriveConstants.kMaxDriveVelocityMetersPerSecond;
      yMove *= Constants.DriveConstants.kMaxDriveVelocityMetersPerSecond;
      rotate *= Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond;

      // Heading controller
      // TODO should be moved before moving cuttoff
      if (headingOverride != null) {
        if (!lastHeadingControllerEnabled) {
          // Reset heading controller if we are entering it for the first time
          headingController.reset(new State(drive.getOdomHeading().getRadians(), 0)); // TODO need to account for angular velocity
        }

        // Calculate the rotate command in Rad/Sec so we can reuse the PID values used in auto
        headingController.setGoal(Math.toRadians(headingOverride));
        rotate = headingController.calculate(drive.getOdomHeading().getRadians());
        rotate = MathUtil.clamp(rotate, -headingControllerConstraints.maxVelocity, headingControllerConstraints.maxVelocity);

        SmartDashboard.putNumber("DriveWithController - Heading goal", headingOverride);
        SmartDashboard.putNumber("DriveWithController - Heading error", Math.toDegrees(headingController.getPositionError()));
      }

      SmartDashboard.putNumber("DriveWithController - xMove", xMove);
      SmartDashboard.putNumber("DriveWithController - yMove", yMove);
      SmartDashboard.putNumber("DriveWithController - rotate", rotate);

      drive.drive(xMove, yMove, rotate, fieldOrient);
    } else {
      // The robot is currently not moving. Check to see if the robot was moving
      if (lastMovingState || latchedModuleStates == null) {
        // The robot was moving and is now moving, so we need to latch the last module states for
        // the "idle"
        // position
        latchedModuleStates =
            new SwerveModuleState[] {
              new SwerveModuleState(0, Rotation2d.fromDegrees((double)lastModuleAngles[0].get(0))),
              new SwerveModuleState(0, Rotation2d.fromDegrees((double)lastModuleAngles[1].get(0))),
              new SwerveModuleState(0, Rotation2d.fromDegrees((double)lastModuleAngles[2].get(0))),
              new SwerveModuleState(0, Rotation2d.fromDegrees((double)lastModuleAngles[3].get(0))),
            };
        SmartDashboard.putNumber(
            "Latched Module Angle 0", latchedModuleStates[0].angle.getDegrees());
      }

      drive.setModuleStates(latchedModuleStates);
    }

    SmartDashboard.putBoolean(
        "DriveWithController - Heading Override enabled", headingOverride != null);
    SmartDashboard.putBoolean("DriveWithController - Moving", moving);
    SmartDashboard.putBoolean("DriveWithController - Field oriented", fieldOrient);
    lastMovingState = moving;
    lastHeadingControllerEnabled = headingOverride != null;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
