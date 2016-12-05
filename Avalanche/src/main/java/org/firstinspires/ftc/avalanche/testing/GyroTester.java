package org.firstinspires.ftc.avalanche.testing;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.avalanche.R;

/**
 * The purpose of this program is to find out how much
 * the gyroscope drifts so we can decide whether or not to apply a low pass filter.
 *
 * Created by Austin on 9/13/2016.
 */
@TeleOp(name = "Gyro Tester", group = "Testing")
@Disabled
public class GyroTester extends LinearOpMode {

    ModernRoboticsI2cGyro gyro;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");

        gyro.calibrate();


        long startTime = System.currentTimeMillis();

        while (gyro.isCalibrating()) {
            telemetry.addData("Calibrating: ", Math.round(((double) (System.currentTimeMillis() - startTime))/1000));
            telemetry.update();

            Thread.sleep(50);
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        MediaPlayer ready = MediaPlayer.create(hardwareMap.appContext, R.raw.imready);
        ready.start();

        waitForStart();

        gyro.resetZAxisIntegrator();

        long startTime = System.currentTimeMillis();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.a) {
                gyro.resetZAxisIntegrator();
                startTime = System.currentTimeMillis();
            }

            double secondsPerOneDegreeDrift = -1;
            double degreesDriftPerSecond = (double) gyro.getHeading() / ((System.currentTimeMillis() - startTime) * 1000);

            if (gyro.getHeading() != 0) {
                secondsPerOneDegreeDrift = (double) (System.currentTimeMillis() - startTime) / gyro.getHeading() / 1000;
            }

            telemetry.addData("heading: ", gyro.getHeading());
            telemetry.addData("drift per second: ", degreesDriftPerSecond);
            telemetry.addData("seconds per degree drift: ", secondsPerOneDegreeDrift);
            telemetry.update();

            idle();
        }
    }
}


