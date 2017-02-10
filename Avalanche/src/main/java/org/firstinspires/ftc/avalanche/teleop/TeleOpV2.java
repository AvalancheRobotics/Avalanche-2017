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
import org.firstinspires.ftc.avalanche.subsystems.MecanumDriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by austinzhang on 2/4/17.
 * This teleop class is built for the generation 2 robot which now utilizes a lift system as well as a mecanum drive train.
 */


@TeleOp(name = "TeleOpV2", group = "TeleOp")
public class TeleOpV2 extends LinearOpMode {

    MecanumDriveTrainController driveTrain;

    DcMotor motorHarvester;

    DcMotor motorShooter;

    DcMotor motorLiftOne;

    DcMotor motorLiftTwo;

    Servo servoLock;

    Servo servoLiftRelease;

    Servo servoSpacerOne;
    Servo servoSpacerTwo;

    int currentLiftStage = 0;


    private void hardwareMapping() {
        driveTrain = new MecanumDriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap), this);


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

        //Initialize lift motors
        motorLiftOne = hardwareMap.dcMotor.get("LiftOne");
        motorLiftTwo = hardwareMap.dcMotor.get("LiftTwo");

        motorLiftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLiftOne.setTargetPosition(ValueStore.SLIDE_STORE);
        motorLiftTwo.setTargetPosition(ValueStore.SLIDE_STORE);

        motorLiftOne.setPower(1);
        motorLiftTwo.setPower(1);


        motorLiftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize lock
        servoLock = hardwareMap.servo.get("Lock");

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        //Initialize lift release
        servoLiftRelease = hardwareMap.servo.get("LiftRelease");
        servoLiftRelease.setPosition(ValueStore.LIFT_HELD);


        //Initialize spacer servos
        servoSpacerOne = hardwareMap.servo.get("SpacerOne");
        servoSpacerTwo = hardwareMap.servo.get("SpacerTwo");

        servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
        servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (!(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left)) {
                driveTrain.manualDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
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

            if (ScaleInput.scale(gamepad2.right_stick_y) != 0 && gamepad2.left_stick_y != 0) {
                if (motorLiftOne.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
                    motorLiftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                motorLiftOne.setPower(gamepad2.left_stick_y);
                motorLiftTwo.setPower(gamepad2.left_stick_y);
            }
            else {
                if (motorLiftOne.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
                    motorLiftOne.setPower(0);
                    motorLiftTwo.setPower(0);
                }
            }

            if (gamepad2.dpad_left) {
                servoLiftRelease.setPosition(ValueStore.LIFT_RELEASED);
            }

            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                if (gamepad2.dpad_up) {
                    currentLiftStage++;
                } else {
                    currentLiftStage--;
                }

                if (currentLiftStage > 2) {
                    currentLiftStage = 0;
                }

                if (motorLiftOne.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                    motorLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (currentLiftStage == 0) {
                    motorLiftOne.setTargetPosition(ValueStore.SLIDE_STORE);
                    motorLiftTwo.setTargetPosition(ValueStore.SLIDE_STORE);
                }

                if (currentLiftStage == 1) {
                    motorLiftOne.setTargetPosition(ValueStore.SLIDE_DRIVE);
                    motorLiftTwo.setTargetPosition(ValueStore.SLIDE_DRIVE);
                }

                if (currentLiftStage == 2) {
                    motorLiftOne.setTargetPosition(ValueStore.SLIDE_CAP);
                    motorLiftTwo.setTargetPosition(ValueStore.SLIDE_CAP);
                }

                motorLiftOne.setPower(-1);
                motorLiftTwo.setPower(-1);

            }

            if (gamepad2.dpad_right) {
                if (servoSpacerOne.getPosition() == ValueStore.SPACER_ONE_STORE) {
                    servoSpacerOne.setPosition(ValueStore.SPACER_ONE_EXTENDED);
                    servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_EXTENDED);
                } else {
                    servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
                    servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);
                }
            }

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
