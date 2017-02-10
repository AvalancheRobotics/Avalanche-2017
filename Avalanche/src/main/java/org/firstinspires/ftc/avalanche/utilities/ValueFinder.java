package org.firstinspires.ftc.avalanche.utilities;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Value Finder", group = "Utilities")
public class ValueFinder extends LinearOpMode {

    Servo servoLock;

    Servo servoLiftRelease;

    Servo servoSpacerOne;
    Servo servoSpacerTwo;


    private DcMotor shooter;
    private DcMotor motorLiftOne;
    private DcMotor motorLiftTwo;

    private int currentIndex;
    private int totalDevices = 6;
    private boolean isServo;
    private String deviceName;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        currentIndex = 0;

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

        shooter = hardwareMap.dcMotor.get("Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPower(0);

        motorLiftOne = hardwareMap.dcMotor.get("LiftOne");
        motorLiftTwo = hardwareMap.dcMotor.get("LiftTwo");
        motorLiftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftOne.setPower(0);
        motorLiftTwo.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            Servo currentServo = null;
            DcMotor currentMotor = null;

            if (gamepad1.a) {
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.right_bumper) {
                currentIndex += 1;
                if (currentIndex >= totalDevices) {
                    currentIndex = 0;
                }
            }

            if (gamepad1.left_bumper) {
                currentIndex -= 1;
                if (currentIndex < 0) {
                    currentIndex = totalDevices - 1;
                }
            }


            if (currentIndex == 0) {
                currentServo = servoLiftRelease;
                isServo = true;
                deviceName = "Lift Release";
            }

            if (currentIndex == 1) {
                currentServo = servoLock;
                isServo = true;
                deviceName = "Servo Lock";
            }

            if (currentIndex == 2) {
                currentServo = servoSpacerOne;
                isServo = true;
                deviceName = "Spacer One";
            }

            if (currentIndex == 3) {
                currentServo = servoSpacerTwo;
                isServo = true;
                deviceName = "Spacer Two";
            }

            if (currentIndex == 4) {
                currentMotor = shooter;
                isServo = false;
                deviceName = "Shooter";
            }

            if (currentIndex == 5) {
                currentMotor = motorLiftOne;
                isServo = false;
                deviceName = "Lift";
            }



            if (isServo) {
                currentServo.setPosition(currentServo.getPosition() + (ScaleInput.scale(gamepad1.left_stick_y) / 50));
                telemetry.addData(deviceName, currentServo.getPosition());
            }
            else {

                if (deviceName.equals("Lift")) {
                    motorLiftOne.setPower(gamepad1.left_stick_y);
                    motorLiftTwo.setPower(gamepad1.left_stick_y);
                    telemetry.addData("motor1: " + motorLiftOne.getCurrentPosition(), "motor2: " + motorLiftTwo.getCurrentPosition());
                }
                else {
                    currentMotor.setPower(ScaleInput.scale(gamepad1.left_stick_y));
                    telemetry.addData(deviceName, currentMotor.getCurrentPosition());
                }
            }

            telemetry.update();

            idle();
        }

    }
}

