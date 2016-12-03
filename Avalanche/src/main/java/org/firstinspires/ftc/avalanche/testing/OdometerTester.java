package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Austin on 9/13/2016.
 *
 */
@TeleOp(name = "Odometer Tester", group = "Testing")
@Disabled
public class OdometerTester extends LinearOpMode {

    DcMotor motor;


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("Harvester");
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        telemetry.addData("Finished mapping", "");
        telemetry.update();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.a) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            telemetry.addData("Odometer: ", motor.getCurrentPosition());

            telemetry.addData("ticksininch", (motor.getCurrentPosition() / 20));

            telemetry.update();

            idle();
        }
    }
}


