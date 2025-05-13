package common;

//import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gyro {

    public enum GyroType {IMU, NAVX}

    GyroType gyroType;
    HardwareDevice device;
    IMU imu = null;
    //AHRS navx = null;

    // for simulated gyro
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private double lastTime = 0;
    private double yaw = 0;
    private int driftDirection = 1;
    private double maxDrift = 2;  //in degrees
    private int sampleRate = 50;  // in hertz

    public Gyro (HardwareMap hardwareMap, String deviceName) {
        /*
        device = hardwareMap.get(deviceName);

        if (device instanceof NavxMicroNavigationSensor) {
            initNavx();

        } else if (device instanceof IMU) {
            if (deviceName.equals(Config.IMU)) {
                initIMU(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            }
            else if (deviceName.equals(Config.IMU_EXPANSION)) {
                initIMU(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            }
        } else  {
            throw new IllegalArgumentException(String.format("Unable to find a gyro with name \"%s\"", deviceName));
        }

        Logger.message("Created gyro %s (%s)", device.getDeviceName(), device.getClass().getSimpleName());
    }

    public Gyro (
            HardwareMap hardwareMap,
            String deviceName,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection) {

        device = hardwareMap.get(deviceName);

        if (device instanceof IMU) {
            initIMU(logoDirection, usbDirection);

        } else if (device instanceof NavxMicroNavigationSensor) {
            initNavx();

        } else {
            throw new IllegalArgumentException(String.format("Unable to find a gyro with name \"%s\"", deviceName));
        }

        Logger.message("Created gyro %s (%s)", device.getDeviceName(), device.getClass().getSimpleName());
    }

    private void initIMU (
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection) {

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        gyroType = GyroType.IMU;
        imu = (IMU) device;
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private void initNavx () {

        gyroType = GyroType.NAVX;
        navx = AHRS.getInstance((NavxMicroNavigationSensor) device, AHRS.DeviceDataType.kProcessedData);

        while ( navx.isCalibrating() ) {
            Thread.yield();
        }
        navx.zeroYaw();
    }

    public void resetYaw() {

        if (gyroType == GyroType.IMU)
            imu.resetYaw();
        else if (gyroType == GyroType.NAVX) {
            navx.zeroYaw();
        }
    }

    public double getYaw() {

        if (gyroType == GyroType.IMU) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);

        } else if (gyroType == GyroType.NAVX) {
            return navx.getYaw();
        }

        return 0;
    }

    public double getYawSimulated () {
        double time = elapsedTime.milliseconds();
        if (time - lastTime >= sampleRate) {
            double increment = maxDrift / sampleRate;
            yaw += (driftDirection * increment);
            if (yaw > maxDrift) {
                yaw = maxDrift;
                driftDirection *= -1;
            } else if (yaw < -maxDrift) {
                yaw = -maxDrift;
                driftDirection *= -1;
            }
            lastTime = time;
        }
        return yaw;
         */
    }

}
