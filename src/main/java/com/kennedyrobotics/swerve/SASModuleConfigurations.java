package com.kennedyrobotics.swerve;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import edu.wpi.first.math.util.Units;

public class SASModuleConfigurations {

    public static final MechanicalConfiguration V1 = new MechanicalConfiguration(
        Units.inchesToMeters(4),
        (12.0 / 40.0) * (20.0/40.0),
        true,
        (1.0/71.0) * (48.0/40.0),
        false
    );

    public static final MechanicalConfiguration V2 = new MechanicalConfiguration(
        Units.inchesToMeters(4),
        (12.0 / 40.0) * (20.0/40.0),
        true,
        (1.0 / 2.0) * (1.0/45.0) * (48.0/40.0), // Belt 2:1 -> Versa Planetary 1:45 ->  Swerve and steer 48:40
        true
    );

    private SASModuleConfigurations() {}
}
