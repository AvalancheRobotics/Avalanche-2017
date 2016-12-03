package org.firstinspires.ftc.avalanche.subsystems;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.utilities.ColorReader;

/**
 * Created by Keith on 9/19/2016.
 * ODOMETER WHEEL NEEDS TO BE IN THE SAME DIRECTION AS THE WHEEL ENCODER
 * Ex. When the odometer wheel ticks increase the wheel encoder ticks increase as well
 * <p/>
 * Updated by Austin on 10/3/2016
 * Incorporated odometer wheel, added gyro and timeout to driveToLine
 * <p/>
 * <p/>
 * TO DO NEXT:
 * Could improve DriveToLine
 * - Instead of giving up after not being able to detect the line, could have the robot instead revert
 * to a preset location
 * for example, if you don't see the line and drive past it, afterwards you could have a preset drive back 5 inches were you think the line is
 * <p/>
 * Also needs testing to figure out whether or not the gyro without a high pass filter is accurate and precise enough.
 */

public class AutoDriveTrainController {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.

    private int initLightLeft;
    private int initLightRight;
    private LinearOpMode linearOpMode;
    private ModernRoboticsI2cGyro gyro;
    private DriveTrainController driveTrain;
    private DcMotor odometer;
    private ColorSensor lineLeft;
    private ColorSensor lineRight;

    private static double P_TURN_COEFF = .01;

    private long startTime;
    private int offset;
    private int drift;


    static final int ODOMETER_INVERSED = -1;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH_ODOMETER = 360;
    static final double COUNTS_PER_INCH_DRIVE_WHEEL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);
    private static final double DRIVE_TO_LINE_SPEED = .1;

    //Example of how to run the AutoDriveTrainController
    /*
    public void init() throws InterruptedException
    {
        //Initialize AutoDriveTrainController
        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, leftBack, rightBack, leftFront, rightFront, odometer);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, 48.0, 0.0);        // Drive FWD 48 inches
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, -45.0);              // Turn  CCW to -45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, -45.0, 0.5);         // Hold -45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 45.0);               // Turn  CW  to  45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 45.0, 0.5);          // Hold  45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 0.0);                // Turn  CW  to   0 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 1.0);           // Hold  0 Deg heading for a 1 second
        autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, -48.0, 0.0);       // Drive REV 48 inches
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 0.5);           // Hold  0 Deg heading for a 1/2 second
        autoDriveTrain.driveToLine(autoDriveTrain.DRIVE_SPEED, 8000);           //Drive at the set speed until 8 seconds has passed or until you reach a white line
    }
    */


    //When adding motors to array list, as a general rule leftBack first, then rightBack, leftFront, and finally rightFront
    public AutoDriveTrainController(ColorSensor lineLeft, ColorSensor lineRight, LinearOpMode linearOpMode, ModernRoboticsI2cGyro gyro, HardwareMap hardwareMap, DcMotor odometer) throws InterruptedException {
        this.lineLeft = lineLeft;
        this.lineRight = lineRight;

        this.lineLeft.enableLed(true);
        Thread.sleep(50);
        this.lineRight.enableLed(true);


        this.linearOpMode = linearOpMode;


        this.gyro = gyro;

        gyro.calibrate();
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        Thread.sleep(5000);
        drift = gyro.getHeading();

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        this.odometer = odometer;

        odometer.setPower(0);

        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveTrain.hardResetEncoders();

        driveTrain.setZeroPowerBehavior(true);

        gyro.resetZAxisIntegrator();


        initLightLeft = this.lineLeft.red() + this.lineLeft.green() + this.lineLeft.blue();
        initLightRight = this.lineRight.red() + this.lineRight.green() + this.lineRight.blue();
    }

    public void callAtBeginningOfOpModeAfterInit() {
        startTime = System.nanoTime();
        offset = gyro.getHeading();
    }


    //Program automatically drives to white line while always facing the same direction
    //Uses gyroscope to maintain the same direction so robot doesn't drift off course

    /**
     * Drive to the white line, used primary for button pressing.
     *
     * @param timeoutMillis Max time in milliseconds you want your robot to drive before
     *                      giving up and stopping.
     * @return boolean              Whether or not the robot reached the line before timeout.
     * @throws InterruptedException If interrupted during idle.
     */
    public boolean driveToLine(double timeoutMillis, boolean forward) throws InterruptedException {
        int negator = -1;

        if (forward) {
            negator = 1;
        }

        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveTrain.setLeftDrivePower(DRIVE_TO_LINE_SPEED * negator);
        driveTrain.setRightDrivePower(DRIVE_TO_LINE_SPEED * negator);

        long startTime = System.currentTimeMillis();

        boolean timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

        while (linearOpMode.opModeIsActive() && !ColorReader.isWhite(initLightLeft,
                lineLeft.red() + lineLeft.green() + lineLeft.blue())
                && ColorReader.isWhite(initLightRight,
                lineRight.red() + lineRight.green() + lineRight.blue())
                && !timeout) {
            // If true this means that we timed out before we detected the white tape, therefore we're not on white.
            timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

            // Allow time for other processes to run.
            linearOpMode.idle();
        }

        // keep looping while we are still active, we haven't taken too long (reached timeout)
        // and we haven't reached the white line yet.

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

        return !timeout;
    }

    //Returns corrected gyro angle
    private int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getIntegratedZValue() - offset - totalDrift;

        return targetHeading;
    }

    public void moveDistanceAtSpeedOnHeading(double speed, double distance, int heading) throws InterruptedException {

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {


            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            int distanceInOdometerTicks = (int) ((distance - .5) * COUNTS_PER_INCH_ODOMETER); //SUBTRACT .5 inches because robot has tendency to overshoot.

            int odometerTarget = (odometer.getCurrentPosition() - distanceInOdometerTicks * ODOMETER_INVERSED);

            int odometerCurrentPosition = odometer.getCurrentPosition();

            odometer.setTargetPosition(odometerTarget);

            driveTrain.setLeftDrivePower(getPower(distance, .1, .08, speed, 0));
            driveTrain.setRightDrivePower(getPower(distance, .1, .08, speed, 0));

            while (!(odometerTarget + 100 > odometerCurrentPosition && odometerTarget - 100 < odometerCurrentPosition)) {
                int distanceLeft = odometerTarget - odometerCurrentPosition;
                odometerCurrentPosition = odometer.getCurrentPosition();

                int odometerTicksToWheelTicks;
                odometerTicksToWheelTicks = (int) (distanceLeft / COUNTS_PER_INCH_ODOMETER * COUNTS_PER_INCH_DRIVE_WHEEL);


                double steer = (heading - getCorrectedHeading()) * P_TURN_COEFF;

                driveTrain.setLeftDrivePower(getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER, .035, .08, speed, -steer));
                driveTrain.setRightDrivePower(getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER, .035, .08, speed, steer));

                driveTrain.setTargetPosition(0, odometerTicksToWheelTicks + driveTrain.getEncoderValue(0));
                driveTrain.setTargetPosition(1, odometerTicksToWheelTicks + driveTrain.getEncoderValue(1));
                driveTrain.setTargetPosition(2, odometerTicksToWheelTicks + driveTrain.getEncoderValue(2));
                driveTrain.setTargetPosition(3, odometerTicksToWheelTicks + driveTrain.getEncoderValue(3));

                linearOpMode.telemetry.addData("leftSpeed", driveTrain.getPower(0));
                linearOpMode.telemetry.addData("rightSpeed", driveTrain.getPower(1));
                linearOpMode.telemetry.addData("o2wticks", odometerTicksToWheelTicks);
                linearOpMode.telemetry.addData("otleft", distanceLeft);
                linearOpMode.telemetry.addData("odomTarget", odometerTarget);
                linearOpMode.telemetry.addData("odomCurrent", odometerCurrentPosition);
                linearOpMode.telemetry.addData("heading", getCorrectedHeading());


                linearOpMode.telemetry.update();


                linearOpMode.idle();
            }

        }

        // Stop all motion;
        driveTrain.setPower(0);
    }

    private double getPower(double inchesFromTarget, double porpConstant, double floor, double ceiling, double steer) {
        double power;

        power = inchesFromTarget * porpConstant;

        int negative = 1;

        if (power < 0) {
            negative = -1;
        }

        power = Math.abs(power);

        power = Math.max(power, floor);

        power = Math.min(power, ceiling);

        power = (power * negative) + (steer * Math.abs(power));

        linearOpMode.telemetry.addData("steer", steer * Math.abs(power));


        return power;
    }

    public void pivotToAngle(int angle, double speed) throws InterruptedException {

        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .07;
        double bottomFloor = -.07;

        int target = angle;


        while (!(heading > target - 2 && heading < target + 2) && linearOpMode.opModeIsActive()) {

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

                linearOpMode.telemetry.addData("turn", "turn left");
                linearOpMode.telemetry.addData("corrected heading", getCorrectedHeading());
            } else {
                driveTrain.setLeftDrivePower(power);
                driveTrain.setRightDrivePower(-power);

                linearOpMode.telemetry.addData("turn", "turn right");
                linearOpMode.telemetry.addData("corrected heading", getCorrectedHeading());
            }

            linearOpMode.telemetry.update();

            heading = getCorrectedHeading();


            linearOpMode.idle();

        }

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

    }

    public void turnUsingOneSide(int angle, double speed) throws InterruptedException {

        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .05;
        double bottomFloor = -.05;

        int target = angle;

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

            if (target > heading) {
                driveTrain.setLeftDrivePower(0);
                driveTrain.setRightDrivePower(power);

                linearOpMode.telemetry.addData("turn", "turn left");
                linearOpMode.telemetry.addData("corrected heading", getCorrectedHeading());
            } else {
                driveTrain.setLeftDrivePower(power);
                driveTrain.setRightDrivePower(0);

                linearOpMode.telemetry.addData("turn", "turn right");
                linearOpMode.telemetry.addData("corrected heading", getCorrectedHeading());
            }

            linearOpMode.telemetry.update();

            heading = getCorrectedHeading();


            linearOpMode.idle();

        }

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

    }

    //Return false if timeout
    public boolean driveToLineAndAlign(int timeoutMillis) throws InterruptedException {
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

        long startTimeOne = System.currentTimeMillis() + 5000;

        while (startTimeOne > System.currentTimeMillis() && (ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()) && ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue()))) {
            linearOpMode.idle();
            linearOpMode.telemetry.addData("1", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
            linearOpMode.telemetry.addData("1", ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue()));
            linearOpMode.telemetry.addData("1", lineRight.red() + lineRight.green() + lineRight.blue());
            linearOpMode.telemetry.addData("1", lineLeft.red() + lineLeft.green() + lineLeft.blue());
            linearOpMode.telemetry.update();
        }

        linearOpMode.telemetry.addData("throughFirstLoop", "2");
        linearOpMode.telemetry.update();

        long startTime = System.currentTimeMillis();

        boolean timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

        boolean leftSideReachedLine = false;
        boolean rightSideReachedLine = false;

        while (linearOpMode.opModeIsActive() && !leftSideReachedLine && !rightSideReachedLine && !timeout) {

            linearOpMode.telemetry.addData("3", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
            linearOpMode.telemetry.addData("3", ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue()));
            linearOpMode.telemetry.addData("3", lineRight.red() + lineRight.green() + lineRight.blue());
            linearOpMode.telemetry.addData("3", lineLeft.red() + lineLeft.green() + lineLeft.blue());
            linearOpMode.telemetry.update();

            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());


            // If true this means that we timed out before we detected the white tape, therefore we're not on white.
            timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

            // Allow time for other processes to run.
            linearOpMode.idle();
        }

        linearOpMode.telemetry.addData("throughSecondLoop", "4");
        linearOpMode.telemetry.update();

        if (timeout) {
            driveTrain.setLeftDrivePower(0);
            driveTrain.setRightDrivePower(0);
            return false;
        }

        if (leftSideReachedLine && rightSideReachedLine) {
            driveTrain.setLeftDrivePower(0);
            driveTrain.setRightDrivePower(0);
            return true;
        }

        if (leftSideReachedLine) {
            linearOpMode.telemetry.addData("leftsidereachedfirst", "5");
            linearOpMode.telemetry.update();

            int startOdometerTicks = odometer.getCurrentPosition();

            while (!rightSideReachedLine) {
                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                linearOpMode.idle();
                linearOpMode.telemetry.addData("6", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
                linearOpMode.telemetry.addData("6", lineRight.red() + lineRight.green() + lineRight.blue());
                linearOpMode.telemetry.update();
            }

            linearOpMode.telemetry.addData("throughThirdLoop", "6");
            linearOpMode.telemetry.update();

            int distBetweenColorSensorsInTicks = odometer.getCurrentPosition() - startOdometerTicks;

            driveTrain.setPower(0);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 0; i < driveTrain.size(); i++) {
                driveTrain.setTargetPosition(i, driveTrain.getEncoderValue(i) - (distBetweenColorSensorsInTicks / 2));
                driveTrain.setPower(-.08);
            }

            while (driveTrain.reachedTargets(3)) {
                linearOpMode.idle();
                for (int i = 0; i < driveTrain.size(); i++) {
                    linearOpMode.telemetry.addData("distTillDone", driveTrain.getTargetPosition(i) - driveTrain.getEncoderValue(i));
                    linearOpMode.telemetry.addData("distTillDone", driveTrain.getTargetPosition(i) - driveTrain.getEncoderValue(i));
                }
                linearOpMode.telemetry.update();
            }

            linearOpMode.telemetry.addData("throughFourthLoop", "7");
            linearOpMode.telemetry.update();

            driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (!(rightSideReachedLine && leftSideReachedLine)) {

                linearOpMode.telemetry.addData("8", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
                linearOpMode.telemetry.addData("8", ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue()));
                linearOpMode.telemetry.addData("8", lineRight.red() + lineRight.green() + lineRight.blue());
                linearOpMode.telemetry.addData("8", lineLeft.red() + lineLeft.green() + lineLeft.blue());
                linearOpMode.telemetry.update();

                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

                driveTrain.setRightDrivePower(.08);
                driveTrain.setLeftDrivePower(-.08);
            }


            int head1 = getCorrectedHeading();

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (rightSideReachedLine && leftSideReachedLine) {
                linearOpMode.idle();

                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
            }

            int head2 = getCorrectedHeading();

            pivotToAngle((head1 + head2) /2, .1);

            driveTrain.setLeftDrivePower(0);
            driveTrain.setRightDrivePower(0);
            return true;
        } else {
            linearOpMode.telemetry.addData("rightsidereachedfirst", "5");
            linearOpMode.telemetry.update();

            int startOdometerTicks = odometer.getCurrentPosition();


            while (!leftSideReachedLine) {
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
                linearOpMode.telemetry.addData("6", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
                linearOpMode.telemetry.addData("6", lineRight.red() + lineRight.green() + lineRight.blue());
                linearOpMode.telemetry.update();
                linearOpMode.idle();
            }

            linearOpMode.telemetry.addData("throughThirdLoop", "6");
            linearOpMode.telemetry.update();

            int distBetweenColorSensorsInTicks = odometer.getCurrentPosition() - startOdometerTicks;

            driveTrain.setPower(0);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            for (int i = 0; i < driveTrain.size(); i++) {
                driveTrain.setTargetPosition(i, driveTrain.getEncoderValue(i) - (distBetweenColorSensorsInTicks / 2));
                driveTrain.setPower(-.08);
            }

            while (driveTrain.reachedTargets(3)) {
                linearOpMode.idle();
                for (int i = 0; i < driveTrain.size(); i++) {
                    linearOpMode.telemetry.addData("distTillDone", driveTrain.getTargetPosition(i) - driveTrain.getEncoderValue(i));
                }
                linearOpMode.telemetry.update();
            }

            linearOpMode.telemetry.addData("throughFourthLoop", "7");
            linearOpMode.telemetry.update();

            driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (!(rightSideReachedLine && leftSideReachedLine)) {

                linearOpMode.telemetry.addData("8", ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue()));
                linearOpMode.telemetry.addData("8", ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue()));
                linearOpMode.telemetry.addData("8", lineRight.red() + lineRight.green() + lineRight.blue());
                linearOpMode.telemetry.addData("8", lineLeft.red() + lineLeft.green() + lineLeft.blue());
                linearOpMode.telemetry.update();


                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

                driveTrain.setRightDrivePower(-.08);
                driveTrain.setLeftDrivePower(.08);
            }

            int head1 = getCorrectedHeading();

            rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
            leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());

            while (rightSideReachedLine && leftSideReachedLine) {
                linearOpMode.idle();

                rightSideReachedLine = ColorReader.isWhite(initLightRight, lineRight.red() + lineRight.green() + lineRight.blue());
                leftSideReachedLine = ColorReader.isWhite(initLightLeft, lineLeft.red() + lineLeft.green() + lineLeft.blue());
            }

            int head2 = getCorrectedHeading();

            pivotToAngle((head1 + head2)/2, .1);


        }
        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);
        return true;
    }


}
