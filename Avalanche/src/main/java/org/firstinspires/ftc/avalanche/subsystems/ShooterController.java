package org.firstinspires.ftc.avalanche.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by austinzhang on 11/4/16.
 */

public class ShooterController {

    DcMotor motor;
    Servo servo;

    public ShooterController(DcMotor shooter, DcMotor lock) {
        motor = shooter;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public void launchOneBall() {

    }

    public void continuiousLaunch() {

    }

    public void rotateShooter() {

    }

    public void update() {
        
    }
}
