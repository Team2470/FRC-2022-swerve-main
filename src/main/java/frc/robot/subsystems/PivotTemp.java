package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotTemp extends SubsystemBase {
  private final WPI_VictorSPX m_leftLeader;
  private final WPI_VictorSPX m_leftFollower;
  private final WPI_VictorSPX m_rightFollowerA;
  private final WPI_VictorSPX m_rightFollowerB;

  private final Encoder m_encoder = new Encoder(0, 1, false, EncodingType.k4X);

  public PivotTemp() {
    NeutralMode neutralMode = NeutralMode.Brake;

    m_leftLeader = new WPI_VictorSPX(20);
    m_leftLeader.setNeutralMode(neutralMode);
    m_leftLeader.configOpenloopRamp(0.5);

    m_leftFollower = new WPI_VictorSPX(21);
    m_leftFollower.follow(m_leftLeader);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.setNeutralMode(neutralMode);

    m_rightFollowerA = new WPI_VictorSPX(22);
    m_rightFollowerA.follow(m_leftLeader);
    m_rightFollowerA.setInverted(InvertType.OpposeMaster);
    m_rightFollowerA.setNeutralMode(neutralMode);

    m_rightFollowerB = new WPI_VictorSPX(23);
    m_rightFollowerB.follow(m_leftLeader);
    m_rightFollowerB.setInverted(InvertType.OpposeMaster);
    m_rightFollowerB.setNeutralMode(neutralMode);

    //                                       4x decoding 1st reduction   2nd Reduction
    //                  1 Encoder Rotation   4 Counts    18 Teeth        10 Teeth   360 Degrees
    // Encoder Counts * ------------------ * --------  * --------      * -------- * ----------
    //                  8192 Counts          1 Cycles    50 Teeth        58 Teeth   1 Rotation

    m_encoder.setDistancePerPulse(
        1.0 / 8192.0 * (4.0 / 1.0) * (18.0 / 50.0) * (10.0 / 58.0) * 360.0);
  }

  public void setPower(double value) {
    if (value < 0 && m_encoder.getDistance() < 2.0 || value > 0 && m_encoder.getDistance() > 180) {
      m_leftLeader.neutralOutput();
      return;
    }

    m_leftLeader.set(ControlMode.PercentOutput, value);
  }

  public void stop() {
    m_leftLeader.neutralOutput();
  }

  public Command setPowerCmd(double value) {
    return Commands.runEnd(() -> setPower(value), () -> stop(), this);
  }

  public void resetEncoder() {
    m_encoder.reset();
  }

  public Command resetEncoderCmd() {
    return Commands.runOnce(this::resetEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Angle", m_encoder.getDistance());
    SmartDashboard.putNumber("Pivot Encoder Raw", m_encoder.getRaw());
    SmartDashboard.putNumber("Pivot Encoder Counts", m_encoder.get());
  }
}
