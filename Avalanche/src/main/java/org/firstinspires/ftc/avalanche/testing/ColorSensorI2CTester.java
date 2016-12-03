package org.firstinspires.ftc.avalanche.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.utilities.ValueStore;


@TeleOp(name = "ColorSensorI2C", group = "Utilities")
public class ColorSensorI2CTester extends LinearOpMode {

    private ColorSensor lineLeft;
    private ColorSensor lineRight;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    private int totalDevices = 4;
    private int currentIndex;
    private String deviceName;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        currentIndex = 0;

        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineRight = hardwareMap.colorSensor.get("lineRight");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");

        lineRight.setI2cAddress(new I2cAddr(0x3c / 2));
        lineLeft.setI2cAddress(new I2cAddr(0x5c / 2));

        colorLeft.setI2cAddress(new I2cAddr(0x6c / 2));
        colorRight.setI2cAddress(new I2cAddr(0x4c / 2));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            ColorSensor currentSensor = null;


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
                currentSensor = colorLeft;
                deviceName = "cleft";
            }

            if (currentIndex == 1) {
                currentSensor = colorRight;
                deviceName = "cright";
            }

            if (currentIndex == 2) {
                currentSensor = lineLeft;
                deviceName = "lleft";
            }

            if (currentIndex == 3) {
                currentSensor = lineRight;
                deviceName = "lright";
            }

            if (currentSensor != null) {
                if (gamepad1.a) {
                    currentSensor.enableLed(true);
                    telemetry.addData("enable Led", "");
                }

                if (gamepad1.b) {
                    currentSensor.enableLed(false);
                    telemetry.addData("disable Led", "");

                }

                telemetry.addData("red", currentSensor.red());
                telemetry.addData("green", currentSensor.green());
                telemetry.addData("blue", currentSensor.blue());
                telemetry.addData("i2c", colorLeft.getI2cAddress().get7Bit());
            }


            telemetry.addData(deviceName, "<- this is our sensor");

            telemetry.update();

            idle();
        }

    }
}

