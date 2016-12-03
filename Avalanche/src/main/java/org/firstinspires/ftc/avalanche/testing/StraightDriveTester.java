package org.firstinspires.ftc.avalanche.testing;


import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;

@TeleOp(name = "Straight Drive Tester", group = "Testing")
@Disabled
public class StraightDriveTester extends LinearOpMode {



    DcMotor odometer;

    DriveTrainController driveTrain;
    long startTime;
    int drift;
    int offset;
    ModernRoboticsI2cGyro gyro;


    static final int ODOMETER_INVERSED = -1;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH_ODOMETER = 360;
    static final double COUNTS_PER_INCH_DRIVE_WHEEL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        odometer = hardwareMap.dcMotor.get("Harvester");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        // Reset encoders
        driveTrain.resetEncoders();

        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        odometer.setPower(0);


        /////////////////////////////////////////
        gyro.calibrate();
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        Thread.sleep(5000);
        drift = gyro.getHeading();
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

            if (gamepad1.a) {
                moveDistanceAtSpeedOnHeading(.6, -10, 0);
            }

            if (gamepad1.x) {
                moveDistanceAtSpeedOnHeading(.6, -20, 0);
            }

            if (gamepad1.b) {
                moveDistanceAtSpeedOnHeading(.6, 20, 0);
            }

            if (gamepad1.y) {
                moveDistanceAtSpeedOnHeading(.6, 10, 0);
            }

            idle();
        }

    }

    //Returns corrected gyro angle
    private int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getIntegratedZValue() - offset - totalDrift;

        return targetHeading;
    }

    public void moveDistanceAtSpeedOnHeading(double speed, double distance, double angle) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int heading = getCorrectedHeading();

            int distanceInOdometerTicks = (int) ((distance - .5) * COUNTS_PER_INCH_ODOMETER); //SUBTRACT .5 inches because robot has tendency to overshoot.

            int odometerTarget = (odometer.getCurrentPosition() - distanceInOdometerTicks * ODOMETER_INVERSED);

            int odometerCurrentPosition = odometer.getCurrentPosition();

            odometer.setTargetPosition(odometerTarget);

            driveTrain.setLeftDrivePower(getPower(distance, .1, .08, speed));
            driveTrain.setRightDrivePower(getPower(distance, .1, .08, speed));

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!(odometerTarget + 100 > odometerCurrentPosition && odometerTarget - 100 < odometerCurrentPosition)) {
                int distanceLeft = odometerTarget - odometerCurrentPosition;
                odometerCurrentPosition = odometer.getCurrentPosition();

                int odometerTicksToWheelTicks;
                odometerTicksToWheelTicks = (int) (distanceLeft / COUNTS_PER_INCH_ODOMETER * COUNTS_PER_INCH_DRIVE_WHEEL);


                double steer = 0;//(heading - getCorrectedHeading()) * P_TURN_COEFF;

                driveTrain.setLeftDrivePower(getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER + steer, .035, .08, speed));
                driveTrain.setRightDrivePower(getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER - steer, .035, .08, speed));

                driveTrain.setTargetPosition(0, odometerTicksToWheelTicks + driveTrain.getEncoderValue(0));
                driveTrain.setTargetPosition(1, odometerTicksToWheelTicks + driveTrain.getEncoderValue(1));
                driveTrain.setTargetPosition(2, odometerTicksToWheelTicks + driveTrain.getEncoderValue(2));
                driveTrain.setTargetPosition(3, odometerTicksToWheelTicks + driveTrain.getEncoderValue(3));

                telemetry.addData("leftSpeed" , driveTrain.getPower(0));
                telemetry.addData("rightSpeed" , driveTrain.getPower(1));
                telemetry.addData("o2wticks" , odometerTicksToWheelTicks);
                telemetry.addData("otleft" , distanceLeft);
                telemetry.addData("odomTarget" , odometerTarget);
                telemetry.addData("odomCurrent" , odometerCurrentPosition);


                telemetry.update();


                idle();
            }

        }

        // Stop all motion;
        driveTrain.setPower(0);
    }

    private double getPower(double inchesFromTarget, double porpConstant, double floor, double ceiling) {
        double power;

        power = inchesFromTarget * porpConstant;

        int negative = 1;

        if (power < 0) {
            negative = -1;
        }

        power = Math.abs(power);

        power = Math.max(power, floor);

        power = Math.min(power, ceiling);

        return power * negative;
    }


}

