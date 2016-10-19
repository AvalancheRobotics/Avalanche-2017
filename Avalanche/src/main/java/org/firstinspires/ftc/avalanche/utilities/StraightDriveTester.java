package org.firstinspires.ftc.avalanche.utilities;


import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

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

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    static final double COUNTS_PER_ODOMETER_REV = 1440 ;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    static final double ODOMETER_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
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

    public void gyroDrive (double speed, double distance, double angle) throws InterruptedException
    {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int totalOdometerTicks;
        int totalDriveWheelTicks;
        int odometerStartValue;
        double  max;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            driveTrain.resetEncoders();

            odometerStartValue = odometer.getCurrentPosition();

            totalOdometerTicks = (int) (distance * COUNTS_PER_INCH_ODOMETER);

            totalDriveWheelTicks = (int) (distance * COUNTS_PER_INCH_DRIVE_WHEEL);

            int ticksTraveledOdometer = odometer.getCurrentPosition() - odometerStartValue;

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = driveTrain.getEncoderValue(0) + totalDriveWheelTicks;
            newRightBackTarget = driveTrain.getEncoderValue(1) + totalDriveWheelTicks;
            newLeftFrontTarget = driveTrain.getEncoderValue(2) + totalDriveWheelTicks;
            newRightFrontTarget = driveTrain.getEncoderValue(3) + totalDriveWheelTicks;

            // Set Target and Turn On RUN_TO_POSITION
            driveTrain.setTargetPosition(0, newLeftBackTarget);
            driveTrain.setTargetPosition(1, newRightBackTarget);
            driveTrain.setTargetPosition(2, newLeftFrontTarget);
            driveTrain.setTargetPosition(3, newRightFrontTarget);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveTrain.setPower(speed);

            // keep looping while we are still active, and odometer has not yet reached its target position.
            // The +- 5 is for allowing a bit of error since we're not going to be able to hit exactly on the odometer ticks
            while (opModeIsActive() && !(ticksTraveledOdometer > totalOdometerTicks - 5 && ticksTraveledOdometer < totalOdometerTicks + 5)){

                ticksTraveledOdometer = odometer.getCurrentPosition() - odometerStartValue;

                //Convert odometer ticks to wheel ticks
                int wheelTicksRemaining = (int) (((totalOdometerTicks - Math.abs(ticksTraveledOdometer)) / COUNTS_PER_INCH_ODOMETER) * COUNTS_PER_INCH_DRIVE_WHEEL);

                // Set Target and Turn On RUN_TO_POSITION

                if (totalOdometerTicks - ticksTraveledOdometer > 0) {
                    driveTrain.setTargetPosition(0, wheelTicksRemaining + driveTrain.getEncoderValue(0));
                    driveTrain.setTargetPosition(1, wheelTicksRemaining + driveTrain.getEncoderValue(1));
                    driveTrain.setTargetPosition(2, wheelTicksRemaining + driveTrain.getEncoderValue(2));
                    driveTrain.setTargetPosition(3, wheelTicksRemaining + driveTrain.getEncoderValue(3));
                }
                else {
                    driveTrain.setTargetPosition(0, -wheelTicksRemaining + driveTrain.getEncoderValue(0));
                    driveTrain.setTargetPosition(1, -wheelTicksRemaining + driveTrain.getEncoderValue(1));
                    driveTrain.setTargetPosition(2, -wheelTicksRemaining + driveTrain.getEncoderValue(2));
                    driveTrain.setTargetPosition(3, -wheelTicksRemaining + driveTrain.getEncoderValue(3));
                }

                // adjust relative speed based on heading error.
                //error = getError(angle);
                steer = getSteer(1, P_DRIVE_COEFF); //error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveTrain.setLeftDrivePower(leftSpeed);
                driveTrain.setRightDrivePower(rightSpeed);

                telemetry.addData("leftSpeed", leftSpeed);
                telemetry.addData("rightSpeed", rightSpeed);
                telemetry.addData("leftSpeed", leftSpeed);
                telemetry.addData("tickstraveled", ticksTraveledOdometer);
                telemetry.addData("totalodomticks", totalOdometerTicks);
                telemetry.addData("wheelticksremaining", wheelTicksRemaining);


                telemetry.update();



                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            driveTrain.setPower(0);
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range
        robotError = targetAngle - getCorrectedHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

