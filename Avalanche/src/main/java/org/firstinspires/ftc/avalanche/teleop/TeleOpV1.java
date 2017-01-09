package org.firstinspires.ftc.avalanche.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.controls.SingleControllerControls;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * Kept for reference only, do not make new changes to this file.
 * <p/>
 * Created by austinzhang on 11/3/16.
 */


@TeleOp(name = "TeleOpV1", group = "TeleOp")
public class TeleOpV1 extends LinearOpMode {

    private ControllerConfig controls;

    DriveTrainController driveTrain;

    DcMotor motorHarvester;

    DcMotor motorShooter;

    Servo servoLock;

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;

    boolean singleController = false;


    private void hardwareMapping() {
        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        driveTrain.reverseMotors();

        //Initialize harvester
        motorHarvester = hardwareMap.dcMotor.get("Harvester");

        motorHarvester.setPower(0);

        motorHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        controls = new SingleControllerControls(gamepad1, gamepad2);

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (!(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left)) {
                driveTrain.manualDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            } else {

                driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (gamepad1.dpad_up) {
                    driveTrain.setLeftDrivePower(.22);
                    driveTrain.setRightDrivePower(.22);
                }

                if (gamepad1.dpad_down) {
                    driveTrain.setLeftDrivePower(-.22);
                    driveTrain.setRightDrivePower(-.22);
                }

                if (gamepad1.dpad_left) {
                    driveTrain.setLeftDrivePower(.22);
                    driveTrain.setRightDrivePower(-.22);
                }

                if (gamepad1.dpad_right) {
                    driveTrain.setLeftDrivePower(-.22);
                    driveTrain.setRightDrivePower(.22);
                }
            }

            if (gamepad1.right_trigger > .8 || gamepad2.right_trigger > .8) {
                motorHarvester.setPower(0); //stop harvester

            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                motorHarvester.setPower(-1); //go backwards with harvester
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                motorHarvester.setPower(1); //go forward with harvester
            }

            if (gamepad2.a || gamepad1.a) {
                servoLock.setPosition(ValueStore.LOCK_LOAD); //load
            }

            if (gamepad2.b || gamepad1.b) {
                servoLock.setPosition(ValueStore.LOCK_RELEASE); //release
            }

            if (ScaleInput.scale(gamepad2.left_trigger) > 0 || ScaleInput.scale(gamepad1.left_trigger) > 0) {
                motorShooter.setPower(Math.abs(Math.max(ScaleInput.scale(gamepad2.left_trigger), ScaleInput.scale(gamepad1.left_trigger))));
            } else {
                motorShooter.setPower(0);
            }

            if (gamepad2.dpad_left) {
                servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
            }

            if (gamepad2.dpad_up) {
                servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

            }

            if (gamepad2.dpad_right) {
                servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
            }

            if (gamepad2.y) {
                telemetry.addData("not drifting", "brake");
                telemetry.update();
                driveTrain.setZeroPowerBehavior(true);
            }
            if (gamepad2.x) {
                telemetry.addData("drifting", "float");
                telemetry.update();
                driveTrain.setZeroPowerBehavior(false);
            }

            servoBeaconShuttle.setPosition((Math.abs(gamepad2.right_stick_y) * (ValueStore.BUTTON_PRESSER_PRESSED - ValueStore.BUTTON_PRESSER_RETRACTED)) + ValueStore.BUTTON_PRESSER_RETRACTED);
        }

        idle();
    }

    private void loadAndLaunch() throws InterruptedException {
        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_RELEASE);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(500);

        launchOneBall();
    }

    private void launchOneBall() throws InterruptedException {
        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setTargetPosition(motorShooter.getCurrentPosition() + ValueStore.ONE_SHOOTER_LOOP);
        motorShooter.setPower(1);
        while (motorShooter.isBusy()) {
            idle();
        }
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
