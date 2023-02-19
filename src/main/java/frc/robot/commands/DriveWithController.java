package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class DriveWithController extends CommandBase {
    private static final double kDeadband = 0.15;

    private final Drivetrain drive;
    private final XboxController controller;

    // State
    private boolean fieldOrient;
    private boolean lastMovingState = false;
    private SwerveModuleState[] latchedModuleStates;
    private final SlewRateLimiter xFilter = new SlewRateLimiter(5);
    private final SlewRateLimiter yFilter = new SlewRateLimiter(5);
    private final SlewRateLimiter rotateFilter = new SlewRateLimiter(5);

    // Keep track of the last 5 module angles
    private static final int kAngleHistoryMilliseconds = 100;
    private static final int kAngleHistoryLength = kAngleHistoryMilliseconds/20;
    private CircularBuffer[] lastModuleAngles = {
        new CircularBuffer(kAngleHistoryLength),
        new CircularBuffer(kAngleHistoryLength),
        new CircularBuffer(kAngleHistoryLength),
        new CircularBuffer(kAngleHistoryLength)
    };

    public DriveWithController(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
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
        double xMove = -controller.getLeftY();
        double yMove = -controller.getLeftX();
        double rotate;

        double left = controller.getLeftTriggerAxis();
        double right = controller.getRightTriggerAxis();

        if (left < right) {
            rotate =  -right;
        } else {
            rotate =  left;
        }
        
        //Apply filter
        xMove = xFilter.calculate(xMove);
        yMove = yFilter.calculate(yMove);
        rotate = rotateFilter.calculate(rotate);

        // Apply deadband
        xMove = MathUtil.applyDeadband(xMove, kDeadband);
        yMove = MathUtil.applyDeadband(yMove, kDeadband);
        rotate = MathUtil.applyDeadband(rotate, kDeadband);

        // Check to for changes between robot and field centric drive modes
        if (controller.getAButton()) {
            fieldOrient = false;
        } else if (controller.getBButton()) {
            fieldOrient = true;
        }

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
            
            if (drive.getSlowMode() || controller.getRightBumper()){
                xMove *= 0.3;
                yMove *= 0.3;
                rotate *= 0.25;
            } else {
                xMove *= 1.0;
                yMove *= 1.0;
                rotate *= 0.5;
            }

            // Now we need to map the percentages to Meters (or Radians) per second, as that is what the drive train
            // subsystem accepts
            xMove *= Constants.Drive.kMaxDriveVelocityMetersPerSecond;
            yMove *= Constants.Drive.kMaxDriveVelocityMetersPerSecond;
            rotate *= Constants.Drive.kMaxAngularVelocityRadiansPerSecond;

            SmartDashboard.putNumber("DriveWithController - xMove", xMove);
            SmartDashboard.putNumber("DriveWithController - yMove", yMove);
            SmartDashboard.putNumber("DriveWithController - rotate", rotate);

            drive.drive(xMove, yMove, rotate, fieldOrient);
        } else {
            // The robot is currently not moving. Check to see if the robot was moving
            if (lastMovingState || latchedModuleStates == null) {
                // The robot was moving and is now moving, so we need to latch the last module states for the "idle"
                // position
                latchedModuleStates = new SwerveModuleState[]{
                    new SwerveModuleState(0, Rotation2d.fromDegrees(lastModuleAngles[0].get(0))),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(lastModuleAngles[1].get(0))),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(lastModuleAngles[2].get(0))),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(lastModuleAngles[3].get(0))),
                };
                SmartDashboard.putNumber("Latched Module Angle 0", latchedModuleStates[0].angle.getDegrees());
            }

            drive.setModuleStates(latchedModuleStates);
        }

        SmartDashboard.putNumber("Last Module Angle Buffer 0[oldest]", lastModuleAngles[0].get(0));
        SmartDashboard.putNumber("Last Module Angle Buffer 0[latest]", lastModuleAngles[0].get(kAngleHistoryLength-1));

        SmartDashboard.putBoolean("DriveWithController - Moving", moving);
        SmartDashboard.putBoolean("DriveWithController - Field oriented", fieldOrient);

        lastMovingState = moving;
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