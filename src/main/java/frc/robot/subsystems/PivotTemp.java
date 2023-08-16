package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotTemp extends SubsystemBase {
    private final WPI_VictorSPX m_leftLeader;
    private final WPI_VictorSPX m_leftFollower;
    private final WPI_VictorSPX m_rightFollowerA;
    private final WPI_VictorSPX m_rightFollowerB;
    

    public PivotTemp() {
        m_leftLeader = new WPI_VictorSPX(20);
        m_leftLeader.setNeutralMode(NeutralMode.Coast);

        m_leftFollower = new WPI_VictorSPX(21);
        m_leftFollower.follow(m_leftLeader);
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_leftFollower.setNeutralMode(NeutralMode.Coast);

        m_rightFollowerA = new WPI_VictorSPX(22);
        m_rightFollowerA.follow(m_leftLeader);
        m_rightFollowerA.setInverted(InvertType.OpposeMaster);
        m_rightFollowerA.setNeutralMode(NeutralMode.Coast);

        m_rightFollowerB = new WPI_VictorSPX(23);
        m_rightFollowerB.follow(m_leftLeader);
        m_rightFollowerB.setInverted(InvertType.OpposeMaster);
        m_rightFollowerB.setNeutralMode(NeutralMode.Coast);
    }

    public void setPower(double value) {
        m_leftLeader.set(ControlMode.PercentOutput,value);
    }

    public void stop() {
        m_leftLeader.neutralOutput();
    }

    public Command setPowerCmd(double value) {
        return Commands.runEnd(
            () -> setPower(value), 
            () -> stop(),
            this
        );
    }
}
