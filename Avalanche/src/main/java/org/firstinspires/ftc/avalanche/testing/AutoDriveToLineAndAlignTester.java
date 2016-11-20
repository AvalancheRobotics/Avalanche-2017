package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ColorReader;

@TeleOp(name = "AD2L&A", group = "Testing")
public class AutoDriveToLineAndAlignTester extends LinearOpMode {
    DriveTrainController driveTrain;
    ColorSensor lineRight;
    ColorSensor lineLeft;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;
    int initLightRight;
    int initLightLeft;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));
        gyro = (ModernRoboticsI2cGyro) (hardwareMap.gyroSensor.get("Gyro"));
        odometer = hardwareMap.dcMotor.get("Odometer");
        lineRight = hardwareMap.colorSensor.get("LineRight");
        lineLeft = hardwareMap.colorSensor.get("LineLeft");

        telemetry.addData("Init", "done");
        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.a) {
                driveToLineAndAlign(10000);
            }

            driveTrain.manualDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

            telemetry.update();
            idle();
        }
    }


    //Return false if timeout
    private boolean driveToLineAndAlign(int timeoutMillis) throws InterruptedException {
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveTrain.setLeftDrivePower(.1);
        driveTrain.setRightDrivePower(.1);

        /** ADD TIMEOUT HERE NEEDS TIMEOUT NEEEEDSSS TIMEOOUTT
         * ATIMAST
         * WAFASF
         * DSAF
         * ASDFADS
         * FSADFQEWBROIUHROPEGRE
         * REG
         *
         */

        while (ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()) && ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue())) {
            idle();
        }

        long startTime = System.currentTimeMillis();

        boolean timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

        boolean leftSideReachedLine = false;
        boolean rightSideReachedLine = false;

        while (opModeIsActive() && !leftSideReachedLine && !rightSideReachedLine && !timeout) {

            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());


            // If true this means that we timed out before we detected the white tape, therefore we're not on white.
            timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

            // Allow time for other processes to run.
            idle();
        }

        if (timeout) {
            return false;
        }

        if (leftSideReachedLine && rightSideReachedLine) {
            return true;
        }

        if (leftSideReachedLine) {

            int startOdometerTicks = odometer.getCurrentPosition();

            while (!rightSideReachedLine) {
                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                idle();
            }

            int distBetweenColorSensorsInTicks = odometer.getCurrentPosition() - startOdometerTicks;

            driveTrain.setPower(0);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 0; i < driveTrain.size(); i++) {
                driveTrain.setTargetPosition(i, driveTrain.getEncoderValue(i) - (distBetweenColorSensorsInTicks / 2));
                driveTrain.setPower(-.08);
            }

            while (driveTrain.isBusy()) {
                idle();
            }

            driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (!(rightSideReachedLine && leftSideReachedLine)) {

                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

                driveTrain.setRightDrivePower(.08);
                driveTrain.setLeftDrivePower(-.08);
            }

        }

        if (rightSideReachedLine) {

            int startOdometerTicks = odometer.getCurrentPosition();

            while (!leftSideReachedLine) {
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
                idle();
            }

            int distBetweenColorSensorsInTicks = odometer.getCurrentPosition() - startOdometerTicks;

            driveTrain.setPower(0);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 0; i < driveTrain.size(); i++) {
                driveTrain.setTargetPosition(i, driveTrain.getEncoderValue(i) - (distBetweenColorSensorsInTicks / 2));
                driveTrain.setPower(-.08);
            }

            while (driveTrain.isBusy()) {
                idle();
            }

            driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (!(rightSideReachedLine && leftSideReachedLine)) {

                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

                driveTrain.setRightDrivePower(-.08);
                driveTrain.setLeftDrivePower(.08);
            }
        }

        return true;
    }

}

