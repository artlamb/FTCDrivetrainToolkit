package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;

/*
 * Test code f
 */
@TeleOp(name="Drive To Line", group="Test")
@Disabled
@SuppressWarnings("unused")
public class DriveToLine extends LinearOpMode {

  private static final double MIN_SPEED = 0.20;

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Press start");
    telemetry.update();

    Drive drive = new Drive(this);
    drive.start();

    telemetry.addLine("a - drive forward to the red line");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      if (gamepad1.a) {
        drive.moveToColor(Drive.COLOR.RED, 1, 0, MIN_SPEED, 0);
        while (gamepad1.a && opModeIsActive()) {
          sleep(10);
        }
      }
    }
  }
}
