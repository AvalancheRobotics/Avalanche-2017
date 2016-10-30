package org.firstinspires.ftc.avalanche.testing;


import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.R;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;

@TeleOp(name = "Straight Drive Tester", group = "TeleOp")
public class StraightDriveTester extends LinearOpMode {


    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    DcMotor odometer;

    DriveTrainController driveTrain;
    long startTime;
    int drift;
    int offset;
    ModernRoboticsI2cGyro gyro;

    MediaPlayer sanic;

    static final int ODOMETER_INVERSED = -1;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    static final double COUNTS_PER_ODOMETER_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double ODOMETER_DIAMETER_INCHES = 2.0;     // For figuring circumference
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH_ODOMETER = (COUNTS_PER_ODOMETER_REV) /
            (ODOMETER_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH_DRIVE_WHEEL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    public static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        odometer = hardwareMap.dcMotor.get("Odometer");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        // Reset encoders
        driveTrain.resetEncoders();

        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odometer.setPower(0);


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

        sanic = MediaPlayer.create(hardwareMap.appContext, R.raw.sanic);
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
                gyroDrive(.6, -10, 0);
            }

            if (gamepad1.x) {
                gyroDrive(.2, -10, 0);
            }

            if (gamepad1.b) {
                gyroDrive(.1, -10, 0);
            }

            if (gamepad1.y) {
                gyroDrive(.6, 10, 0);
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

    public void gyroDrive(double speed, double distance, double angle) throws InterruptedException {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int heading = getCorrectedHeading();

            int distanceInOdometerTicks = (int) (distance * COUNTS_PER_INCH_ODOMETER);

            int odometerTarget = -(odometer.getCurrentPosition() - distanceInOdometerTicks * ODOMETER_INVERSED);

            int odometerCurrentPosition = odometer.getCurrentPosition();

            odometer.setTargetPosition(odometerTarget);

            driveTrain.setLeftDrivePower(getPower(distance, .1, .08, speed));
            driveTrain.setRightDrivePower(getPower(distance, .1, .08, speed));

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!(odometerTarget + 10 > odometerCurrentPosition && odometerTarget - 10 < odometerCurrentPosition)) {
                int distanceLeft = odometerTarget - odometerCurrentPosition;
                odometerCurrentPosition = odometer.getCurrentPosition();

                int odometerTicksToWheelTicks;
                odometerTicksToWheelTicks = (int) (distanceLeft / COUNTS_PER_INCH_ODOMETER * COUNTS_PER_INCH_DRIVE_WHEEL);


                double steer = 0;//(heading - getCorrectedHeading()) * P_TURN_COEFF;

                driveTrain.setLeftDrivePower(-getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER + steer, .1, .08, speed));
                driveTrain.setRightDrivePower(-getPower(distanceLeft / COUNTS_PER_INCH_ODOMETER - steer, .1, .08, speed));

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

