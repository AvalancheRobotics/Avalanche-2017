package org.firstinspires.ftc.avalanche.testing;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.avalanche.R;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

@TeleOp(name = "Turn Tester", group = "Testing")
public class TurnTester extends LinearOpMode {


    DriveTrainController driveTrain;
    long startTime;
    int drift;
    int offset;
    ModernRoboticsI2cGyro gyro;

    MediaPlayer sanic;


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        // Reset encoders
        driveTrain.resetEncoders();


        /////////////////////////////////////////
        gyro.calibrate();                //
        //
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        //
        Thread.sleep(5000);                    //
        drift = gyro.getHeading(); //
        /////////////////////////////////////////

        telemetry.addData("Done Mapping", "finished.");

        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        startTime = System.nanoTime();
        offset = gyro.getHeading();

        // Go go gadget robot!
        while (opModeIsActive()) {

            driveTrain.setLeftDrivePower(ScaleInput.scale(gamepad1.left_stick_y));
            driveTrain.setRightDrivePower(ScaleInput.scale(gamepad1.right_stick_y));

            if (gamepad1.dpad_down) {
                sanic.stop();
            }

            if (gamepad1.a) {
                telemetry.addData("turn 180", "start");
                telemetry.update();
                pivotToAngle(180, .4);
                telemetry.addData("turn 180", "fin");
                telemetry.update();
            }

            if (gamepad1.b) {
                telemetry.addData("turn 90", "start");
                telemetry.update();
                pivotToAngle(90, .4);
                telemetry.addData("turn 90", "fin");
                telemetry.update();
            }

            if (gamepad1.y) {
                telemetry.addData("turn 0", "start");
                telemetry.update();
                pivotToAngle(0, .4);
                telemetry.addData("turn 0", "fin");
                telemetry.update();
            }

            if (gamepad1.x) {
                telemetry.addData("turn 270", "start");
                telemetry.update();
                pivotToAngle(270, .4);
                telemetry.addData("turn 270", "fin");
                telemetry.update();
            }

            telemetry.addData("corrected heading" , getCorrectedHeading());

            telemetry.addData("heading" , gyro.getHeading());


            telemetry.addData("int z", gyro.getIntegratedZValue());


            telemetry.update();

            idle();
        }
    }

    public void pivotToAngle(int angle, double speed) throws InterruptedException {


        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .05;
        double bottomFloor = -.05;

        int target = angle;

        while (! (heading > target - 1 && heading < target + 1) ) {

            long currentTime = System.currentTimeMillis();

            power = Math.abs((target - heading) * proportionalConst);

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;

            if (target > heading) {
                driveTrain.setLeftDrivePower(-power);
                driveTrain.setRightDrivePower(power);

                telemetry.addData("turn", "turn left");
                telemetry.addData("corrected heading" , getCorrectedHeading());
            }
            else {
                driveTrain.setLeftDrivePower(power);
                driveTrain.setRightDrivePower(-power);

                telemetry.addData("turn", "turn right");
                telemetry.addData("corrected heading" , getCorrectedHeading());
            }

            telemetry.update();

            heading = getCorrectedHeading();


            idle();

        }

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

    }

    /** TURN TO ANGLE AUTOMATICALLY FAST AS POSSIBLE IN BETA NOT WORKING*/
    /* public void pivotToAngle(int angle, double speed) throws InterruptedException {
        int heading = getCorrectedHeading();

        long lastTime = System.currentTimeMillis();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .05;
        double bottomFloor = -.05;

        int target = angle;
        while (target > 359)
            target = target - 360;
        while (target < 0)
            target = target + 360;

        while (!(heading > target - 1 && heading < target + 1)) {

            long currentTime = System.currentTimeMillis();

            power = Math.abs((target - heading) * proportionalConst);

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;


            boolean tarGreater = target - heading > 0;

            if ((tarGreater && target - heading > 180) || (!tarGreater && target - heading < 180)) {
                driveTrain.setRightDrivePower(-power);
                driveTrain.setLeftDrivePower(power);

                telemetry.addData("Turn", "Right");
            } else {
                driveTrain.setRightDrivePower(power);
                driveTrain.setLeftDrivePower(-power);
                telemetry.addData("Turn", "Left");
            }

            telemetry.addData("target-heading", target - heading);

            telemetry.addData("power: " , power);

            telemetry.addData("angle: " , heading);

            telemetry.addData("loopTime",  currentTime - lastTime);

            telemetry.addData("right power: ", driveTrain.getPower(1));

            telemetry.addData("left power: ", driveTrain.getPower(0));

            telemetry.update();

            heading = getCorrectedHeading();

            lastTime = currentTime;

            idle();
        }

        driveTrain.setRightDrivePower(0);
        driveTrain.setLeftDrivePower(0);
    }*/

    //Returns corrected gyro angle
    private int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getIntegratedZValue() - offset - totalDrift;

        return targetHeading;
    }

}

