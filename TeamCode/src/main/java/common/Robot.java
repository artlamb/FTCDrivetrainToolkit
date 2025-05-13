package common;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    public Drive      drive = null;
    public Vision     vision = null;

    public Servo leftHook;
    public Servo rightHook;
    public Servo specimen;

    /* Declare OpMode members. */
    private final LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initialize the Robot
     */
    public void init() {

        Logger.addLine("Mycelium Engine v3.9.8");

        drive = new Drive(opMode);

        try {

            /*dropper = opMode.hardwareMap.get(Servo.class, Config.PIXEL_DROPPER);

            droneAngle = opMode.hardwareMap.get(Servo.class, Config.DRONE_ANGLE);
            droneFire = opMode.hardwareMap.get(Servo.class, Config.DRONE_FIRE);

            pixelIntake = opMode.hardwareMap.get(DcMotor.class, Config.PIXEL_INTAKE);
            intakeRotate = opMode.hardwareMap.get(Servo.class, Config.INTAKE_ROTATE);
            spinnerGray = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_GRAY);
            spinnerBlack = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BLACK);
            spinnerBucket = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BUCKET);*/


        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }
    }

    public void startVision () {
        //vision = new Vision(opMode);
    }

    public void turn(double degrees) {
        drive.turn(degrees);
    }

    public void forward (double distance) {
        drive.forward(distance);
    }

    public void forwardSlow (double distance) {
        drive.forwardSlow(distance);
    }

    public void forwardFast (double distance) {
        drive.forwardFast(distance);
    }

    public void back (double distance) {
        drive.back(distance);
    }

    public void backFast (double distance) {
        drive.backFast(distance);
    }

    public void strafeLeft (double distance) {
        drive.strafeLeft(distance);
    }

    public void strafeRight (double distance) {
        drive.strafeRight(distance);
    }

} // end of class

