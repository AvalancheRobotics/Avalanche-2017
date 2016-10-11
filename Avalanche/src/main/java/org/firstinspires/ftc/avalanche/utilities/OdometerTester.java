package org.firstinspires.ftc.avalanche.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Austin on 9/13/2016.
 *
 */
@TeleOp(name = "Odometer Tester", group = "Utilities")
public class OdometerTester extends LinearOpMode {

    DcMotor motor;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("Odometer");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        telemetry.addData("Finished mapping", "");
        telemetry.update();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            telemetry.addData("Odometer: ", motor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}


