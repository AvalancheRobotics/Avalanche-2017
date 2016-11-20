package org.firstinspires.ftc.avalanche.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Nicholas on 10/23/16.
 */

public class DoubleControllerControls implements ControllerConfig
{
    Gamepad gamepad1;
    Gamepad gamepad2;
    public DoubleControllerControls(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public float LTrack() {return gamepad1.left_stick_y;}
    public float RTrack() {return gamepad1.right_stick_y;}
    public boolean AutoLeftButtonPresserButtonPressed() {return gamepad2.dpad_left;}
    public boolean AutoRightButtonPresserButtonPressed() {return gamepad2.dpad_right;}
    public boolean HarvesterButtonPressed() {return gamepad2.a;}
    public boolean LoadAndShootButtonPressed() {return gamepad2.y;}
    public boolean turnLeft() {return gamepad1.left_bumper;}
    public boolean turnRight() {return gamepad1.right_bumper;}
    public boolean modifierKey() {return gamepad1.dpad_down;}
    public boolean increaseSpeed() {return gamepad1.b;}
    public boolean decreaseSpeed() {return gamepad1.x;}
    public boolean reverse() {return gamepad1.a;}
    public boolean forward() {return gamepad1.y;}
    public boolean ReverseHarvesterButtonPressed() {return gamepad2.b;}
    public boolean LoadPositionButtonPressed() {return gamepad2.left_bumper;}
    public boolean ReleasePositionButtonPressed() {return gamepad2.right_bumper;}
    public float ShootOneBall() {return gamepad2.right_trigger;}
    public boolean ExtendButtonPresser() {return gamepad2.right_stick_y > 0.5;}
    public boolean RetractButtonPresser() {return gamepad2.right_stick_y < -0.5;}
    public boolean TiltButtonPresserLeftUp() {return gamepad2.right_stick_x < -0.5;}
    public boolean TiltButtonPresserRightUp() {return gamepad2.right_stick_x > 0.5;}
    public boolean HoldButtonPresserPosition() {return gamepad2.right_stick_button;}
}
