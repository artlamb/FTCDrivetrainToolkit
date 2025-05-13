/*
 * This this contains the class that controls the robot's drive train
 */

package common;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import main.MainTeleOp;

public class Drive extends Thread {

    static final boolean LOG_VERBOSE = false;

    public static double DRIVE_FACTOR  = 0.635;    // 0.97;
    public static double STRAFE_FACTOR = 1.2;      // 1.11;
    public static double TURN_FACTOR   = 13.38;     // 13.38;

    // Drive train
    private final double COUNTS_PER_MOTOR_REV = 28;              // Gobilda 5203 Yellow Jacket
    private final double DRIVE_GEAR_REDUCTION = 19.2;              // Gearing
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches
    private final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final double RAMP_DISTANCE = COUNTS_PER_INCH * 6;   // ramp down distance in encoder counts
    private final double RAMP_TIME = 100;                       // ramp up time in milliseconds
    private final double RAMP_MIN_SPEED = 0.4;

    private final double MIN_SPEED = 0.40;
    private final double MAX_SPEED = 1;
    private final double MAX_ROTATE_SPEED = 1;

    public enum DIRECTION { FORWARD, BACK, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, DRIVER, STOOPED }

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 1.75F;

    public enum COLOR {RED, BLUE, WHITE};

    //  Drive train motors
    public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel

    private IMU imu;
    private NormalizedColorSensor colorSensor = null;

    public DistanceSensor distanceSensor;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    private boolean running = true;
    private boolean driving = false;

    private int leftFrontStartPos  = 0;
    private int rightFrontStartPos = 0;
    private int leftBackStartPos   = 0;
    private int rightBackStartPos  = 0;

    private DIRECTION lastDirection = DIRECTION.STOOPED;

    public List<DcMotorEx> motors;
    LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    /**
     * Initialize the drive train motors.
     */
    private void init() {
        initIMU();

        try {
            leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            leftBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            colorSensor.setGain(COLOR_SENSOR_GAIN);

            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);

        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

         for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initIMU () {
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //imu.resetYaw();
    }

    /**
     * Drive the robot with gamepad 1 joysticks one a separate thread
     */
    public void run() {

        while (!opMode.isStarted()) Thread.yield();
        Logger.message("robot drive thread started");

        ElapsedTime driveTime = new ElapsedTime();
        double lastTime = driveTime.milliseconds();
        double lastSpeed = 0;
        double accelerationPerMS = (MAX_SPEED - MIN_SPEED) / (1000 * 1.5);   // 1.5 second to accelerate to full speed
        double decelerationPerMS = (MAX_SPEED - MIN_SPEED) / (1000);     // 1 second to come to full stop

        while (running && opMode.opModeIsActive()) {

            double x = 0;  // Reduce drive rate to 50%.
            double y = 0;  // Reduce strafe rate to 66%. (was 50%)
            double yaw = 0;  // Reduce rotate rate to 33%.
            double speed = 0;
            // Left stick to go forward back and strafe. Right stick to rotate. Left trigger accelerate.
            Gamepad gamepad1 = opMode.gamepad1;
            x = gamepad1.left_stick_y / 1;  // Reduce drive rate to 100% (was 50%).
            y = gamepad1.left_stick_x / 1;  // Reduce strafe rate to 100%. (was 66%)
            yaw = -gamepad1.right_stick_x / 1;  // Reduce rotate rate to 100% (was 33%).
            speed = (gamepad1.left_trigger * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;

            // limit acceleration and deceleration to prevent skidding.
            double currentTime = driveTime.milliseconds();
            if (!driving) {
                speed = MIN_SPEED;
            } else {
                double deltaTime = currentTime - lastTime;
                double acceleration = (speed - lastSpeed) / (deltaTime);
                if ((speed > lastSpeed) && (acceleration > (accelerationPerMS * deltaTime)))
                    speed = lastSpeed + (accelerationPerMS * deltaTime);
                else if ((speed < lastSpeed) && (acceleration < decelerationPerMS * deltaTime)) {  // not currently used
                    speed = lastSpeed - (decelerationPerMS * deltaTime);
                }
            }
            lastTime = currentTime;
            lastSpeed = speed;

            if (speed > MAX_SPEED) speed = MAX_SPEED;
            if (x == 0 && y == 0 && yaw != 0) {
                if (speed > MAX_ROTATE_SPEED) speed = MAX_ROTATE_SPEED;
            }

            if (x != 0 || y != 0 || yaw != 0) {
                DIRECTION direction;
                double MAX_STICK = 0.5;

                /*if (yaw != 0) {
                    direction = DIRECTION.DRIVER;
                } else if (Math.abs(x) == MAX_STICK || (x != 0 && y == 0)) {
                    if (x > 0)
                        direction = DIRECTION.FORWARD;
                    else
                        direction = DIRECTION.BACK;
                } else if (Math.abs(y) == MAX_STICK || (x == 0 /*&& y != 0 *//*)) {
                    if (y > 0)
                        direction = DIRECTION.LEFT;
                    else
                        direction = DIRECTION.RIGHT;
                } else {
                    direction = DIRECTION.DRIVER;
                }*/

                direction = DIRECTION.DRIVER;

                if (direction == DIRECTION.DRIVER) {
                    Logger.message("mainTO mode " + MainTeleOp.mode);
                    moveRobot(x, y, yaw, speed);
                } else {
                    moveRobot(direction, speed);
                }

                driving = true;
                lastDirection = direction;
                //Logger.message("%-12s   %6.2f %6.2f %6.2f  %6.2f   %6.2f ", direction, x , y, yaw, gamepad1.left_trigger, speed);

            } else if (driving) {
                stopRobot();
                lastDirection = DIRECTION.STOOPED;
                driving = false;

            } else {
                Thread.yield();
            }
        }
        Logger.message("robot drive thread stopped");
    }

    /**
     * Stop the thread's run method
     */
    public void end () {
        running = false;
    }

    /**
     * Move robot according to desired axes motions
     *
     * @param x   Positive is forward
     * @param y   Positive is strafe left
     * @param yaw Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {

        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void moveRobot(double x, double y, double yaw, double speed) {

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        if (x == 0 && y == 0 && yaw == 0 ) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;

        } else {
            leftFrontPower = x - y - yaw;
            rightFrontPower = x + y + yaw;
            leftBackPower = x + y - yaw;
            rightBackPower = x - y + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // ToDo remove, done in run
            /*if (speed == 0)
                speed = MIN_SPEED;
            else if (speed > MAX_SPEED) {
                speed = MAX_SPEED;
            }
            if (x == 0 && y == 0 && yaw != 0) {
                if (speed > MAX_ROTATE_SPEED)
                    speed = MAX_ROTATE_SPEED;
            }*/

            double scale = (1 / max) * speed;

            leftFrontPower *= scale;
            rightFrontPower *= scale;
            leftBackPower *= scale;
            rightBackPower *= scale;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    /**
     *  Method to move the robot in the specified direction. The encoder are used the correct for drift
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     */
    public void moveRobot(DIRECTION direction, double speed) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = 1;
            rightFrontSign =  -1;
            leftBackSign   = 1;
            rightBackSign  =  -1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  -1;
            rightFrontSign = 1;
            leftBackSign   =  -1;
            rightBackSign  = 1;
        }

        // If the direction have changed get the encoder positions.
        if (direction != lastDirection) {
            leftFrontStartPos = leftFrontDrive.getCurrentPosition();
            rightFrontStartPos = rightFrontDrive.getCurrentPosition();
            leftBackStartPos = leftBackDrive.getCurrentPosition();
            rightBackStartPos = rightBackDrive.getCurrentPosition();
        }

        // Correct for drift
        double leftFrontPos =  Math.abs(leftFrontDrive.getCurrentPosition()  - leftFrontStartPos);
        double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition() - rightFrontStartPos);
        double leftBackPos =   Math.abs(leftBackDrive.getCurrentPosition()   - leftBackStartPos);
        double rightBackPos =  Math.abs(rightBackDrive.getCurrentPosition()  - rightBackStartPos);
        double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

        double scale = .0015;
        double leftFrontAdjust = (maxPos - leftFrontPos) * scale;
        double rightFrontAdjust = (maxPos - rightFrontPos) * scale;
        double leftBackAdjust = (maxPos - leftBackPos) * scale;
        double rightBackAdjust = (maxPos - rightBackPos) * scale;

        /*double leftFrontPower = (speed + leftFrontAdjust) * leftFrontSign;
        double rightFrontPower = (speed + rightFrontAdjust) * rightFrontSign;
        double leftBackPower = (speed + leftBackAdjust) * leftBackSign;
        double rightBackPower = (speed + rightBackAdjust) * rightBackSign;*/

        double leftFrontPower = (speed) * leftFrontSign;
        double rightFrontPower = (speed) * rightFrontSign;
        double leftBackPower = (speed) * leftBackSign;
        double rightBackPower = (speed) * rightBackSign;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.3f %4.3f %4.3f %4.3f     position: %6.0f %6.0f %6.0f %6.0f",
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower,
                    leftFrontAdjust,
                    rightFrontAdjust,
                    leftBackAdjust,
                    rightBackAdjust,
                    leftFrontPos,
                    rightFrontPos,
                    leftBackPos,
                    rightBackPos);
        }
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */

    public void moveDistance(DIRECTION direction, double speed, double inches, double timeout) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = 1;
            rightFrontSign =  -1;
            leftBackSign   =  -1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  -1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  =  -1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = 1;
            rightFrontSign =  -1;
            leftBackSign   = 1;
            rightBackSign  =  -1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  -1;
            rightFrontSign = 1;
            leftBackSign   =  -1;
            rightBackSign  = 1;
        }

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for drift
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            //double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramUp = speedRange + RAMP_MIN_SPEED;
            //double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            //double rampPower = Math.min(Math.min(ramUp, ramDown), speed);
            double rampPower = Math.min(ramUp, speed);

            double scale = .0015;
            double leftFrontAdjust = (maxPos - leftFrontPos) * scale;
            double rightFrontAdjust = (maxPos - rightFrontPos) * scale;
            double leftBackAdjust = (maxPos - leftBackPos) * scale;
            double rightBackAdjust = (maxPos - rightBackPos) * scale;

            /*leftFrontDrive.setPower((rampPower + leftFrontAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust) * rightBackSign);*/

            leftFrontDrive.setPower(rampPower * leftFrontSign);
            rightFrontDrive.setPower(rampPower * rightFrontSign);
            leftBackDrive.setPower(rampPower * leftBackSign);
            rightBackDrive.setPower(rampPower * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE)
                Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     velocity: %4.2f %4.2f %4.2f %4.2f",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.getVelocity(),
                        rightFrontDrive.getVelocity(),
                        leftBackDrive.getVelocity(),
                        rightBackDrive.getVelocity());
        }

        // Stop all motion;

        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
    }

    /*public void moveDistance(DIRECTION direction, double speed, double inches, double timeout) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = 1;
            rightFrontSign =  -1;
            leftBackSign   =  -1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  -1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  =  -1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = 1;
            rightFrontSign =  -1;
            leftBackSign   = 1;
            rightBackSign  =  -1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  -1;
            rightFrontSign = 1;
            leftBackSign   =  -1;
            rightBackSign  = 1;
        }

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for drift
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            //double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramUp = speedRange + RAMP_MIN_SPEED;
            //double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            //double rampPower = Math.min(Math.min(ramUp, ramDown), speed);
            double rampPower = Math.min(ramUp, speed);

            double scale = .0015;
            double leftFrontAdjust = (maxPos - leftFrontPos) * scale;
            double rightFrontAdjust = (maxPos - rightFrontPos) * scale;
            double leftBackAdjust = (maxPos - leftBackPos) * scale;
            double rightBackAdjust = (maxPos - rightBackPos) * scale;

            /*leftFrontDrive.setPower((rampPower + leftFrontAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust) * rightBackSign);*//*

            leftFrontDrive.setPower((rampPower) * leftFrontSign);
            rightFrontDrive.setPower((rampPower) * rightFrontSign);
            leftBackDrive.setPower((rampPower) * leftBackSign);
            rightBackDrive.setPower((rampPower) * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE)
                Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     velocity: %4.2f %4.2f %4.2f %4.2f",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.getVelocity(),
                        rightFrontDrive.getVelocity(),
                        leftBackDrive.getVelocity(),
                        rightBackDrive.getVelocity());
        }

        // Stop all motion;

        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
    }/*

    /**
     * Move the robot until the specified color is detected.
     *
     * @param color the color to detect
     * @param x positive for forward, negative for backwards
     * @param y positive for strafe left ???, negative for strafe right ???
     * @param speed drive speed
     * @param timeout timeout in milliseconds
     */
    public boolean moveToColor(COLOR color, double x, double y, double speed, double timeout){

        boolean found = false;
        float[] hsvValues = new float[3];
        ElapsedTime elapsedTime = new ElapsedTime();

        resetEncoders();
        moveRobot(x, y, 0, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive())  {
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV color space
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            //Logger.message("hue %f saturation %f", hue, saturation);
            if (color == COLOR.BLUE) {
                if (hue >= 190 && hue <= 230 && saturation >= .5) {
                    Logger.message("blue line found");
                    found = true;
                }
            } else if (color == COLOR.RED) {
                if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                    Logger.message("red line found");
                    found = true;
                }
            }
            //if (maxInches && (maxInches <= getDistanceTraveled())) {
            //    Logger.warning("no line found, traveled %5.2f inches", getDistanceTraveled());
            //    break;
            //}
            if (elapsedTime.milliseconds() > timeout){
                Logger.warning("timeout, no line found, traveled %5.2f inches", getDistanceTraveled());
                break;
            }
        }
        stopRobot();
        return found;
    }

    /**
     * Move forward until the distance sensor detects an object at the specified distance
     *
     * @param inches  distance for the object to stop
     * @param speed   speed to move forward
     * @param timeout timeout in milliseconds
     * @return true if an object was detected at the specified distance
     */
    public boolean moveToObject (double inches, double speed, double timeout) {

        // If manually driving, exit
        if (driving) return false;

        ElapsedTime elapsedTime = new ElapsedTime();
        int count = 0;
        double startDistance = 0;
        double average;
        boolean found = false;

        resetEncoders();
        moveRobot(DIRECTION.FORWARD, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (count == 0) {
                average = 0;
                startDistance = distance;
            } else {
                average = (startDistance - distance) / count;
                //Logger.message("distance %6.2f  %6.2f", distance, average);
            }
            count++;

            if (distance - inches <= average / 2) {
                stopRobot();
                //Logger.message("object found, distance %6.2f ", distance);
                found = true;
 //           } else if ( distance - inches < 2) {
 //               moveRobot(1, 0, 0, 0.1);
            }

            // return if timed out, of manually driving
            if (elapsedTime.milliseconds() > timeout || driving) {
                stopRobot();
                Logger.warning("no object found, traveled %6.2f inches", getDistanceTraveled());
                break;
            }
        }

        return found;
    }

    /**
     * Stop all the drive train motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        lastDirection = DIRECTION.STOOPED;
    }

    public void setBraking(boolean enabled) {

        for (DcMotor motor : motors) {
            if (enabled)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void resetEncoders() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(mode);
        }
    }

    public double encoderTicksPerInch() {
        return (25.017783857729079);

    }
    public List<Double> getWheelPositions() {

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add((double)position / encoderTicksPerInch());
        }
        return wheelPositions;
    }

    public double getDistanceTraveled() {
        double traveled = 0;

        for (DcMotor motor : motors)
            traveled += motor.getCurrentPosition();
        return traveled / motors.size() / encoderTicksPerInch();
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in degrees in a range of -180 to 180
     */
    public double getOrientation(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetOrientation() {
        imu.resetYaw();
    }

    public void forward (double distance) {
        moveDistance(DIRECTION.FORWARD,0.6, distance, 0);
    }

    public void forwardSlow (double distance) {
        moveDistance(DIRECTION.FORWARD,0.2, distance, 0);
    }

    public void forwardFast (double distance) {
        moveDistance(DIRECTION.FORWARD,1, distance, 0);
    }

    public void back (double distance) {
        moveDistance(DIRECTION.BACK,0.6, distance, 0);
    }

    public void backFast (double distance) {
        moveDistance(DIRECTION.BACK,1, distance, 0);
    }

    public void strafeLeft (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= STRAFE_FACTOR;
        moveDistance(DIRECTION.LEFT,0.7, distance, 0);
    }

    public void strafeRight (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= STRAFE_FACTOR;
        moveDistance(DIRECTION.RIGHT,0.7, distance, 0);
    }

    public void turn(double degrees) {

        double circumference = 2 * Math.PI * 18.3003819;
        double inches = Math.abs(degrees) / 360 * circumference;
        if (degrees > 0)
            moveDistance(DIRECTION.TURN_LEFT, 1,  inches, 0 );
        else
            moveDistance(DIRECTION.TURN_RIGHT, 1,  inches, 0 );
    }

    public void turnWithIMU(double degrees) {
        imu.resetYaw();
        opMode.sleep(200);
        if (degrees > 0) {
            moveRobot(0, 0, 1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current >= degrees-1)
                    break;
            }
        } else if (degrees < 0) {
            moveRobot(0, 0, -1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current <= degrees+1)
                    break;
            }
        }
        stopRobot();
    }
    /*
        @param x inches forward and back (+ and -)
        @param y inches right and left (+ and -)
        @param yaw degrees right and left (+ and -)
        @param speed (0 to 1)
     */
    public void moveComp(float x, float y, float yaw, float speed) {
        // Calculate the distance the robot should travel based on `yaw`
        // This distance is related to the amount of rotation the robot should perform.
        double circumference = 2 * Math.PI * 18.3003819;  // Circumference of the robot's wheel path
        double turnInches = (yaw / 360) * circumference;   // Convert yaw (degrees) to inches for rotation

        // Calculate the encoder ticks per inch (how far each encoder travels per inch)
        double tpi = encoderTicksPerInch();

        // Compute the movement components for forward/backward and turning
        float forwardPower = y * speed;    // Forward/backward (y axis movement)
        float strafePower = x * speed;     // Strafe (x axis movement)
        float turnPower = (float) (turnInches * tpi);  // Turn amount converted to encoder ticks

        // Compute the individual wheel powers
        float leftFrontPower = forwardPower - strafePower - turnPower;
        float rightFrontPower = forwardPower + strafePower + turnPower;
        float leftBackPower = forwardPower + strafePower - turnPower;
        float rightBackPower = forwardPower - strafePower + turnPower;

        // Apply the scaling based on encoder ticks per inch (this will give precise control over distance)
        leftFrontPower *= tpi;
        rightFrontPower *= tpi;
        leftBackPower *= tpi;
        rightBackPower *= tpi;

        // Reset the encoders and set mode for the motors
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set the target positions for each motor
        leftFrontDrive.setTargetPosition((int) leftFrontPower);
        leftBackDrive.setTargetPosition((int) leftBackPower);
        rightFrontDrive.setTargetPosition((int) rightFrontPower);
        rightBackDrive.setTargetPosition((int) rightBackPower);

        // Debugging the wheel powers
        Logger.addLine(Double.toString(leftFrontPower));
        Logger.addLine(Double.toString(rightFrontPower));
        Logger.addLine(Double.toString(leftBackPower));
        Logger.addLine(Double.toString(rightBackPower));

        // Set the motor power (speed controls the speed, not the distance)
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Set the motors to run to the target position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach the target positions
        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            Logger.addLine(Float.toString(leftFrontDrive.getCurrentPosition()));
        }
    }

} // end of class

