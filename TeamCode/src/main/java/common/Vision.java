/*
 * This file contains support for TensorFlow object recognition and AprilTag recognition
 */
package common;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Vision {
    //all calibration files are preset on the limelight, ignore controls for now
    Limelight3A limelight;

    private LinearOpMode opMode;

    public enum pipelines { RED, BLUE, YELLOW }

    public Vision(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init() {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        opMode.telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
    }

    public void limelightState(boolean active, int pipeline) {
        limelight.pipelineSwitch(pipeline);
        if (active) {
            limelight.start();
        } else {
            limelight.stop();
        }
    }

    //public void getResult()

}
