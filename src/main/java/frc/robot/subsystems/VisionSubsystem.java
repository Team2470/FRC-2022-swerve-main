package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionIO;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
  private static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);

  private final VisionIO[] cameras;
  private final VisionIO.VisionIOInputs[] inputs;

  private final List<VisionSubsystem.PoseAndTimestamp> results = new ArrayList<>();

  private int acceptableTagID;
  private boolean useSingleTag = false;

  /**
   * Initializes cameras and input loggers
   *
   * @param cameras Array of cameras being used
   */
  public VisionSubsystem(VisionIO... cameras) {
    this.cameras = cameras;
    inputs = new VisionIO.VisionIOInputs[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      inputs[i] = new VisionIO.VisionIOInputs();
    }

    // TODO configure a vision tab in ShuffleBoard
  }

  @Override
  public void periodic() {
    // Logger.getInstance().recordOutput("useSingleTag", useSingleTag);

    // clear results from last periodic
    results.clear();

    for (int i = 0; i < inputs.length; i++) {
      // update and process new inputs
      cameras[i].updateInputs(inputs[i]);
      // Logger.getInstance().processInputs("Vision/" + cameras[i].getName() + "/Inputs",
      // inputs[i]);

      // TODO figure out what the FMS sends, before robot match starts.
      // 2910 had: !DriverStation.isAutonomous()
      // Going to try to accept updates all the time. This should maybe settable/toggle by the driver or elsewhere to control
      // when to accept updates
      if (true) {
        SmartDashboard.putBoolean("Vision accepting updates", true);
        if (inputs[i].hasTarget && inputs[i].isNew && inputs[i].maxDistance < LOWEST_DISTANCE && inputs[i].maxAmbiguity < 0.4) {
          if (useSingleTag) {
            if (inputs[i].singleIDUsed == acceptableTagID) {
              processVision(i);
            }
          } else {
            processVision(i);
          }
        }  
      } else {
        SmartDashboard.putBoolean("Vision accepting updates", false);
      }
    }

    // Logger.getInstance().recordOutput("Vision/ResultCount", results.size());
  }

  public void processVision(int cameraNum) {
    // create a new pose based off the new inputs
    Pose2d currentPose =
        new Pose2d(
            inputs[cameraNum].x, inputs[cameraNum].y, new Rotation2d(inputs[cameraNum].rotation));
    // Logger.getInstance().recordOutput(cameras[cameraNum].getName() + " pose", currentPose);

    // add the new pose to a list
    results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
  }

  /** Returns the last recorded pose */
  public List<VisionSubsystem.PoseAndTimestamp> getVisionOdometry() {
    return results;
  }

  /** Returns the array of known cameras */
  public VisionIO[] getCameras() {
    return cameras;
  }

  /** Inner class to record a pose and its timestamp */
  public static class PoseAndTimestamp {
    Pose2d pose;
    double timestamp;

    public PoseAndTimestamp(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }

    public Pose2d getPose() {
      return pose;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }

  public void setUseSingleTag(boolean useSingleTag) {
    setUseSingleTag(useSingleTag, 0);
  }

  public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
    this.useSingleTag = useSingleTag;
    this.acceptableTagID = acceptableTagID;
  }

  public void setReferencePose(Pose2d pose) {
    for (VisionIO io : cameras) {
      io.setReferencePose(pose);
    }
  }

  public double getMinDistance(int camera) {
    return inputs[camera].minDistance;
  }

  public void takeSnapshot() {
    for (VisionIO io : cameras) {
      io.takeSnapshot();
    }
  }

  public Command takeSnapshotCommand() {
    return Commands.runOnce(this::takeSnapshot);
  }
}
