package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive.ModuleConfig;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Drivetrain extends SubsystemBase {
   //: Helpers

   //: Hardware
   private final Pigeon2 m_imu;
   private final SwerveModule[] m_swerve_modules = new SwerveModule[4];

   public Drivetrain() {
      //: IMU setup
      m_imu = new Pigeon2(
         Constants.Drive.kPigeonID, 
         Constants.Drive.kPigeonCANBus.bus_name
      );

      ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
      var imuShuffleboard = tab.getLayout("IMU", BuiltInLayouts.kList)
         .withSize(2, 2).withPosition(8, 0);
      
      imuShuffleboard.addNumber
         ( "Heading", () -> getHeading().getDegrees() );

      Mk4ModuleConfiguration moduleConfig = Mk4ModuleConfiguration.getDefaultSteerNEO();
      moduleConfig.setNominalVoltage(Constants.Drive.kDriveVoltageCompensation);

      //: Swerve setup
      this.m_swerve_modules[0] = this.createModule(
         Constants.Drive.kFrontLeft,
         moduleConfig, tab
      );
      this.m_swerve_modules[1] = this.createModule(
         Constants.Drive.kFrontRight,
         moduleConfig, tab
      );
      this.m_swerve_modules[2] = this.createModule(
         Constants.Drive.kBackLeft,
         moduleConfig, tab
      );
      this.m_swerve_modules[3] = this.createModule(
         Constants.Drive.kBackRight,
         moduleConfig, tab
      );
   }

   private SwerveModule createModule(ModuleConfig config, Mk4ModuleConfiguration moduleConfig, ShuffleboardTab tab) {
      return Mk4SwerveModuleHelper.createNeo(
         tab.getLayout(config.name, BuiltInLayouts.kList)
            .withSize(2, 6).withPosition(config.col, config.line),
         
         moduleConfig, Mk4SwerveModuleHelper.GearRatio.L2,
         config.drivingID, config.steeringID, //: drving & steering IDs
         config.encoderID, config.offset.getRadians() //: encoder ID and offset (rotation)
      );
   }

   public void setModuleStates(SwerveModuleState[] states) {
      for (int i = 0; i < states.length; i ++) {
         states[i] = SwerveModuleState.optimize(
            states[i], new Rotation2d(
               this.m_swerve_modules[i].getSteerAngle()
            )
         );

         this.m_swerve_modules[i].set(
            states[i].speedMetersPerSecond / 
               Constants.Drive.kMaxDriveVelocityMetersPerSecond * 
               Constants.Drive.kDriveVoltageCompensation,
            states[i].angle.getRadians()
         );
      }
   }

   public SwerveModuleState[] getModuleStates() {
      // Note the order of modules needs to match the order provided to DriveConstants.kDriveKinematics
      return new SwerveModuleState[]{
          new SwerveModuleState(m_swerve_modules[0].getDriveVelocity(), new Rotation2d(m_swerve_modules[0].getSteerAngle())),
          new SwerveModuleState(m_swerve_modules[1].getDriveVelocity(), new Rotation2d(m_swerve_modules[1].getSteerAngle())),
          new SwerveModuleState(m_swerve_modules[2].getDriveVelocity(), new Rotation2d(m_swerve_modules[2].getSteerAngle())),
          new SwerveModuleState(m_swerve_modules[3].getDriveVelocity(), new Rotation2d(m_swerve_modules[3].getSteerAngle()))
      };
    }

   public void drive(double xSpeed, double ySpeed, double rotation, boolean feildRelative) {
      ChassisSpeeds chassisSpeeds;

      if (feildRelative)
         { chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getHeading());}
      else
         { chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotation); }

      setModuleStates(Constants.Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
   }

   public void stop() {
      drive(0, 0, 0, false);
   }

   @Override public void periodic() {}

   public Rotation2d getHeading() {
      return Rotation2d.fromDegrees(this.m_imu.getYaw());
   }

   public void resetHeading() {
      this.m_imu.setYaw(0);
   }
}