package org.firstinspires.ftc.avalanche.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.MotorController;
import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;

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

    private final double LOAD_LOCK = .2; //ARBITRARY VALUE

    private final double RELEASE_LOCK = .8; //ARBITRARY VALUE

    private final int ONE_SHOOTER_LOOP = 1120; //ARBITRARY VALUE


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

        servoLock.setPosition(RELEASE_LOCK);

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
                servoLock.setPosition(LOAD_LOCK);
            }

            if (gamepad2.b) {
                servoLock.setPosition(RELEASE_LOCK);
            }


            //Load Lock and Launch Ball
            if (gamepad2.y) {
                servoLock.setPosition(LOAD_LOCK);

                servoLock.setPosition(RELEASE_LOCK);

                launchOneBall();
            }

            idle();
        }
    }

    private void launchOneBall() {
    }

}
