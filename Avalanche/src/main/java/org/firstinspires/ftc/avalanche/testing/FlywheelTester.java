package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Austin on 9/13/2016.
 */
@TeleOp(name = "Flywheel Tester", group = "Utilities")
public class FlywheelTester extends LinearOpMode {

    DcMotor motor;
    DcMotor motor2;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        motor2 = hardwareMap.dcMotor.get("motor2");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        telemetry.addData("Finished mapping", "");
        telemetry.update();

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.a) {
                motor.setPower(1);
                motor2.setPower(1);
            }

            if (gamepad1.b) {
                motor.setPower(0);
                motor2.setPower(0);
            }

            if (gamepad1.x) {
                motor.setPower(-1);
                motor2.setPower(-1);
            }

            if (gamepad1.dpad_up) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.dpad_down) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad1.y) {
                if (motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT)
                    telemetry.addData("zero power", "float");
                else
                    telemetry.addData("zero power", "brake");
                telemetry.update();
            }

            idle();
        }
    }
}


