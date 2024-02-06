package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConfig;
import java.util.HashMap;
import java.util.List;

public class Drivetrain extends SubsystemBase {

  private static final double ESTIMATION_COEFFICIENT = 0.025;

  // : Helpers
  private final SwerveDrivePoseEstimator m_odometry;
  // private final SwerveDriveOdometry m_odometry; // TODO make this selectable
  private final Field2d m_field = new Field2d();
  private final VisionSubsystem m_vision;

  // : Hardware
  private final Pigeon2 m_imu;
  private final SwerveModule[] m_swerve_modules = new SwerveModule[4];

  private boolean m_slowMode;

  public boolean getSlowMode() {
    return m_slowMode;
  }

  public void setSlowMode(boolean slowMode) {
    m_slowMode = slowMode;
  }

  public Drivetrain(VisionSubsystem vision) {
    m_vision = vision;

    // : IMU setup
    m_imu =
        new Pigeon2(
            Constants.DriveConstants.kPigeonID, Constants.DriveConstants.kPigeonCANBus.bus_name);
    m_imu.configEnableCompass(false);

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    var imuShuffleboard =
        tab.getLayout("IMU", BuiltInLayouts.kList).withSize(2, 2).withPosition(8, 0);

    imuShuffleboard.addNumber("Heading", () -> getIMUHeading().getDegrees());
    imuShuffleboard.addNumber("Pitch", () -> m_imu.getPitch());
    imuShuffleboard.addNumber("Roll", () -> m_imu.getRoll());

    Mk4ModuleConfiguration moduleConfig = Mk4ModuleConfiguration.getDefaultSteerNEO();
    moduleConfig.setNominalVoltage(Constants.DriveConstants.kDriveVoltageCompensation);
    moduleConfig.setDriveCurrentLimit(40);

    // : Swerve setup
    this.m_swerve_modules[0] =
        this.createModule(Constants.DriveConstants.kFrontLeft, moduleConfig, tab);
    this.m_swerve_modules[1] =
        this.createModule(Constants.DriveConstants.kFrontRight, moduleConfig, tab);
    this.m_swerve_modules[2] =
        this.createModule(Constants.DriveConstants.kBackLeft, moduleConfig, tab);
    this.m_swerve_modules[3] =
        this.createModule(Constants.DriveConstants.kBackRight, moduleConfig, tab);

    // Setup odometry
    m_odometry =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(1.5, 1.5, 1.5)); // 7028 uses 1.5

    var odometryTab =
        tab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 2).withPosition(10, 0);

    odometryTab.addNumber("X (inches)", () -> Units.metersToInches(getPose().getX()));
    odometryTab.addNumber("Y (inches)", () -> Units.metersToInches(getPose().getY()));
    odometryTab.addNumber("Theta (degrees)", () -> getPose().getRotation().getDegrees());

    tab.add("Field", m_field).withSize(5, 4).withPosition(8, 2);
  }

  private SwerveModule createModule(
      ModuleConfig config, Mk4ModuleConfiguration moduleConfig, ShuffleboardTab tab) {
    return Mk4iSwerveModuleHelper.createKrakenNeo(
        tab.getLayout(config.name, BuiltInLayouts.kList)
            .withSize(2, 6)
            .withPosition(config.col, config.line),
        moduleConfig,
        Mk4iSwerveModuleHelper.GearRatio.L2,
        config.drivingID,
        config.steeringID, // : drving & steering IDs
        config.encoderID,
        config.offset.getRadians() // : encoder ID and offset (rotation)
        );
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      states[i] =
          SwerveModuleState.optimize(
              states[i], new Rotation2d(this.m_swerve_modules[i].getSteerAngle()));

      this.m_swerve_modules[i].set(
          states[i].speedMetersPerSecond
              / Constants.DriveConstants.kMaxDriveVelocityMetersPerSecond
              * Constants.DriveConstants.kDriveVoltageCompensation,
          states[i].angle.getRadians());
    }
  }

  public void resetSteerEncoders() {
    for (int i = 0; i < 4; i++) {
      ((CANCoder) m_swerve_modules[i].getSteerEncoder().getInternal()).setPositionToAbsolute();
    }
  }

  public void setNominalVoltages(double voltage) {
    for (SwerveModule swerveModule : m_swerve_modules) {
      ((CANSparkMax) swerveModule.getDriveMotor()).enableVoltageCompensation(voltage);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    // Note the order of modules needs to match the order provided to
    // DriveConstants.kDriveKinematics
    return new SwerveModuleState[] {
      new SwerveModuleState(
          m_swerve_modules[0].getDriveVelocity(),
          new Rotation2d(m_swerve_modules[0].getSteerAngle())),
      new SwerveModuleState(
          m_swerve_modules[1].getDriveVelocity(),
          new Rotation2d(m_swerve_modules[1].getSteerAngle())),
      new SwerveModuleState(
          m_swerve_modules[2].getDriveVelocity(),
          new Rotation2d(m_swerve_modules[2].getSteerAngle())),
      new SwerveModuleState(
          m_swerve_modules[3].getDriveVelocity(),
          new Rotation2d(m_swerve_modules[3].getSteerAngle()))
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    // Note the order of modules needs to match the order provided to
    // DriveConstants.kDriveKinematics
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          m_swerve_modules[0].getDriveDistance(),
          new Rotation2d(m_swerve_modules[0].getSteerAngle())),
      new SwerveModulePosition(
          m_swerve_modules[1].getDriveDistance(),
          new Rotation2d(m_swerve_modules[1].getSteerAngle())),
      new SwerveModulePosition(
          m_swerve_modules[2].getDriveDistance(),
          new Rotation2d(m_swerve_modules[2].getSteerAngle())),
      new SwerveModulePosition(
          m_swerve_modules[3].getDriveDistance(),
          new Rotation2d(m_swerve_modules[3].getSteerAngle()))
    };
  }

  public void drive(double xSpeed, double ySpeed, double rotation, boolean feildRelative) {
    ChassisSpeeds chassisSpeeds;

    if (feildRelative) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getOdomHeading());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
    }

    setChassisSpeeds(chassisSpeeds);
  }

  private void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  @Override
  public void periodic() {

    // Update robot pose
    m_odometry.update(getIMUHeading(), getModulePositions());

    // Feed in Vision measurements
    for (int i = 0; i < m_vision.getVisionOdometry().size(); i++) {
      m_odometry.addVisionMeasurement(
          m_vision.getVisionOdometry().get(i).getPose(),
          m_vision.getVisionOdometry().get(i).getTimestamp(),
          VecBuilder.fill(
              m_vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
              m_vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
              5.0));
    }
    m_vision.setReferencePose(getPose());

    m_field.setRobotPose(getPose());

    SmartDashboard.putBoolean("Slow Mode", m_slowMode);
  }

  public Rotation2d getIMUHeading() {
    return Rotation2d.fromDegrees(this.m_imu.getYaw());
  }

  public Rotation2d getOdomHeading() {
    return getPose().getRotation();
  }

  public boolean isLevel() {
    return Math.abs(m_imu.getRoll()) < 5;
  }

  public double getRoll() {
    return m_imu.getRoll();
  }

  public void resetHeading() {
    // this.m_imu.setYaw(0);
    resetOdometry(new Pose2d());
  }

  /**
   * Return the currently-estimated pose of the robot
   *
   * @return the pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }
  /**
   * Resets the odometry to the specified pose
   *
   * @param pose the pose to switch to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getIMUHeading(), getModulePositions(), pose);
  }

  //
  // Command Helpers
  //

  public Command xStop() {
    return new RunCommand(
        () -> {
          var latchedModuleStates =
              new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
              };
          this.setModuleStates(latchedModuleStates);
        },
        this);
  }

  // : uses drivetrain member
  /** DOES NOT WORK CURRENTLY WITH NEW PATHPLANNER */
  public Command createAutoPath(HashMap<String, Command> events, String pathFile, PathConstraints pathConstraints) {
    AutoBuilder.configureHolonomic(
      this::getPose, this::resetOdometry, 
      () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()),
      this::setChassisSpeeds, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(5d,0d,0d),
        new PIDConstants(2d,0d,0d),
        3.5, //: max speed (m/s)
        0.4, //: distance between modules and center of robot
        new ReplanningConfig()
      ), () -> {
        var alliances = DriverStation.getAlliance();
        if (alliances.isPresent()) {
          return alliances.get() == DriverStation.Alliance.Red;
        } return false;
      }, this);

      return AutoBuilder.buildAuto(pathFile);
  }
  // public Command createAutoPath(
  //     HashMap<String, Command> eventMap, String pathName, PathConstraints pathConstraints) {
  //   List<PathPlannerTrajectory> pathGroup = PathPlannerPath.fromChoreoTrajectory(pathName, pathConstraints);

  //   SwerveAutoBuilder autoBuilder =
  //       new SwerveAutoBuilder(
  //           this::getPose,
  //           this::resetOdometry,
  //           Constants.DriveConstants.kDriveKinematics,
  //           AutoConstants.kPIDTranslation,
  //           AutoConstants.kPIDRotation,
  //           this::setModuleStates,
  //           eventMap,
  //           true,
  //           this);

  //   return autoBuilder.fullAuto(pathGroup);
  // }
}
