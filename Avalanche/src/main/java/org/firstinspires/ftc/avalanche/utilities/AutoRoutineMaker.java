package org.firstinspires.ftc.avalanche.utilities;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;

/**
 * Created by austinzhang on 11/30/16.
 */
@TeleOp(name = "AutoMaker", group = "Utilities")
public class AutoRoutineMaker extends LinearOpMode {
    DriveTrainController driveTrain;

    DcMotor motorHarvester;

    DcMotor motorShooter;

    Servo servoLock;

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;

    private ModernRoboticsI2cGyro gyro;

    long startTime;

    int offset;

    int drift;

    static final double COUNTS_PER_INCH_ODOMETER = 360;

    boolean singleController = false;


    private void hardwareMapping() throws InterruptedException {
        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        driveTrain.reverseMotors();

        //Initialize harvester
        motorHarvester = hardwareMap.dcMotor.get("Harvester");

        motorHarvester.setPower(0);

        motorHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorHarvester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize shooter
        motorShooter = hardwareMap.dcMotor.get("Shooter");

        motorShooter.setPower(0);

        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize lock
        servoLock = hardwareMap.servo.get("Lock");

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        //Initialize beacon servos
        servoBeaconShuttle = hardwareMap.servo.get("BeaconShuttle");

        servoBeaconShuttle.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);

        servoBeaconTilt = hardwareMap.servo.get("BeaconTilt");

        servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

        gyro = (ModernRoboticsI2cGyro) (hardwareMap.gyroSensor.get("Gyro"));


        gyro.calibrate();
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        Thread.sleep(5000);
        drift = gyro.getHeading();


        telemetry.addData("Done", "Calibrating");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        startTime = System.nanoTime();
        offset = gyro.getHeading();


        // Go go gadget robot!
        while (opModeIsActive()) {
            if (ScaleInput.scale(gamepad1.left_trigger) != 0 || ScaleInput.scale(gamepad1.right_trigger) != 0 || gamepad1.right_bumper || gamepad1.left_bumper) {

                if (ScaleInput.scale(gamepad1.left_trigger) != 0) {
                    double power = ScaleInput.scale(gamepad1.left_trigger);
                    driveTrain.setLeftDrivePower(-power);
                    driveTrain.setRightDrivePower(-power);
                }

                if (ScaleInput.scale(gamepad1.right_trigger) != 0) {
                    double power = ScaleInput.scale(gamepad1.right_trigger);
                    driveTrain.setLeftDrivePower(power);
                    driveTrain.setRightDrivePower(power);
                }

                if (gamepad1.left_bumper) {
                    driveTrain.setLeftDrivePower(-.15);
                    driveTrain.setRightDrivePower(.15);
                }

                if (gamepad1.right_bumper) {
                    driveTrain.setLeftDrivePower(.15);
                    driveTrain.setRightDrivePower(-.15);
                }

            }
            else {
                driveTrain.setLeftDrivePower(0);
                driveTrain.setRightDrivePower(0);
            }



            if (gamepad1.a) {
                motorHarvester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.dpad_up) {
                telemetry.addData("Shooter", "Firing");
                motorShooter.setPower(1);
            } else {
                motorShooter.setPower(0);
            }

            telemetry.addData("Inches Traveled: ", ((double) Math.round(motorHarvester.getCurrentPosition() / COUNTS_PER_INCH_ODOMETER * 100)) / 100);

            telemetry.addData("Current Angle: ", getCorrectedHeading());

            telemetry.update();
        }
    }

    //Returns corrected gyro angle
    private int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getIntegratedZValue() - offset - totalDrift;

        return targetHeading;
    }

}
