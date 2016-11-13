package org.firstinspires.ftc.avalanche.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.controls.DefaultControls;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * Created by austinzhang on 11/3/16.
 */


@TeleOp(name = "TeleOpV1", group = "TeleOp")
public class TeleOpV1 extends LinearOpMode{

    private ControllerConfig controls;

    DriveTrainController driveTrain;

    DcMotor motorHarvester;

    DcMotor motorShooter;

    Servo servoLock;

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;


    private void hardwareMapping() {
        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

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

        controls = new DefaultControls(gamepad1, gamepad2);

        // Go go gadget robot!
        while (opModeIsActive()) {
            driveTrain.manualDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

            if (gamepad1.right_trigger > .7) {
                motorHarvester.setPower(0);
            }

            if (gamepad1.left_bumper) {
                motorHarvester.setPower(-1);
            }

            if (gamepad1.right_bumper) {
                motorHarvester.setPower(1);
            }

            if (gamepad2.a) {
                servoLock.setPosition(ValueStore.LOCK_LOAD);
            }

            if (gamepad2.b) {
                servoLock.setPosition(ValueStore.LOCK_RELEASE);
            }


            //Load Lock and Launch Ball
            if (gamepad2.y) {
                loadAndLaunch();
            }

            if (ScaleInput.scale(gamepad2.left_trigger) > 0) {
                motorShooter.setPower(ScaleInput.scale(gamepad2.left_trigger));
            }
            else {
                motorShooter.setPower(0);
            }

            idle();
        }
    }

    private void loadAndLaunch() throws InterruptedException {
        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_RELEASE);

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
