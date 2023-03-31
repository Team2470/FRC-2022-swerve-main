package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

public class TOF {
   private TimeOfFlight m_tFlight;

   public TOF() {
      m_tFlight = new TimeOfFlight(0);
   }

   public double distanceFromTarget() {
      if (m_tFlight.isRangeValid())
         return m_tFlight.getRange();
      
      return 0.0;
   }

   public boolean isRangeValid() {
      return m_tFlight.isRangeValid();
   }

   public boolean targetDetected() {
      return distanceFromTarget() >= 100;
   }

   public void kill() {
      m_tFlight.close();
   }
}