package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive2Dock extends CommandBase {
   private final double goal = 0;
   private boolean isFinished;
   private Drivetrain m_drivetrain;

   public Drive2Dock(Drivetrain drivetrain) {
      m_drivetrain = drivetrain;
   }

   @Override public void initialize() {
      isFinished = false;
   }

   @Override void execute() {
      double roll = m_drivetrain.getRoll();
      double kP = 
   }
}