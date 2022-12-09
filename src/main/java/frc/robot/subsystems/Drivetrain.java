package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
      
      imuShuffleboard.addNumber(
         "Heading", () -> getHeading().getDegrees()
      );

      Mk4ModuleConfiguration moduleConfiguration = new Mk4ModuleConfiguration();
      moduleConfiguration.setNominalVoltage(Constants.Drive.kDriveVoltageCompensation);

      //: Swerve setup
   }

   @Override public void periodic() {}

   public Rotation2d getHeading() {
      return Rotation2d.fromDegrees(this.m_imu.getYaw());
   }

   public void resetHeading() {
      this.m_imu.setYaw(0);
   }

}