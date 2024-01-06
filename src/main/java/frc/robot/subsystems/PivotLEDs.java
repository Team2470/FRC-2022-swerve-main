package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionIO;

public class PivotLEDs extends SubsystemBase {
    public enum Side {
        kLeft("Left"),
        kRight("Right");

        public final String name;
        private Side(String name) {
            this.name = name;
        }
    } 

    public enum PivotLEDState {
        kCameraDisconnected,
        kCameraConnected,
        kCamera1TagDetected,
        kCamera2TagDetected,
        kCamera3OrMoreTagDetected,
    }

    private final Side m_side;
    private final String m_name;
    private final VisionIO m_camera;

    public PivotLEDs(Side side, VisionIO camera) {
        m_side = side;
        m_name = "PivotLED-"+m_side.name;
        m_camera = camera;
    }

    @Override
    public void periodic() {
        // Determine the led state
        PivotLEDState state = PivotLEDState.kCameraDisconnected;
        int detectedTags = m_camera.detectedTagCount(); 
        if (m_camera.isConnected()) {
            state = PivotLEDState.kCameraConnected;
            if (detectedTags == 1) {
                state = PivotLEDState.kCamera1TagDetected;
            } else if (detectedTags == 2) {
                state = PivotLEDState.kCamera2TagDetected;
            } else if (detectedTags >=3) {
                state = PivotLEDState.kCamera3OrMoreTagDetected;
            }
        }

        // Set LEDS
        // TODO

        SmartDashboard.putString(m_name, state.toString());
    }
}
