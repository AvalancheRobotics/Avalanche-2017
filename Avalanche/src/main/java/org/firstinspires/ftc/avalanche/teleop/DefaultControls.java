package org.firstinspires.ftc.avalanche.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;

/**
 * Created by Nicholas on 10/23/16.
 */

public class DefaultControls implements ControllerConfig
{
    Gamepad gamepad1;
    Gamepad gamepad2;
    public DefaultControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public float LTrack() {return gamepad1.left_stick_y;}
    public float RTrack() {return gamepad1.right_stick_y;}
    public boolean LeftButtonPresserButtonPressed() {return gamepad2.dpad_left;}
    public boolean RightButtonPresserButtonPressed() {return gamepad2.dpad_right;}
    public boolean HarvesterButtonPressed() {return gamepad2.a;}
    public boolean ShooterButtonPressed() {return gamepad2.y;}
    public boolean turnLeft() {return gamepad1.left_bumper;}
    public boolean turnRight() {return gamepad1.right_bumper;}
    public boolean modifierKey() {return gamepad1.dpad_down;}
    public boolean increaseSpeed() {return gamepad1.b;}
    public boolean decreaseSpeed() {return gamepad1.x;}
    public boolean reverse() {return gamepad1.a;}
    public boolean forward() {return gamepad1.y;}
}
