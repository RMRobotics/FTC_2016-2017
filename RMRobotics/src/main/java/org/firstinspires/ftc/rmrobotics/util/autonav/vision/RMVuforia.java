package org.firstinspires.ftc.rmrobotics.util.autonav.vision;

import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl;

public class RMVuforia extends VuforiaLocalizerImpl {
    public RMVuforia(Parameters parameters) {
        super(parameters);
    }

    // Pause and release camera
    public void RMPause() {
        pauseAR();
    }

    // Resume Vuforia
    public void RMResume() {
        resumeAR();
    }
}
