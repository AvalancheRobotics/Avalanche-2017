package org.firstinspires.ftc.avalanche.utilities;

/**
 * Created by Nicholas on 10/9/16.
 */
import android.media.MediaPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;

public class ControllerManip {
    Gamepad gamepad1;
    Gamepad gamepad2;
    //Whenever a button on the gamepad is used in a method, in can be replaced at the driver's discretion.
    public ControllerManip(Gamepad gamepad1, Gamepad gamepad2)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        //gamepad1 is used for driving, while gamepad2 is used for any other functionality.
    }

    //returns a value between -1.0 and 1.0
    public float LTrack() { return gamepad1.left_stick_y; }

    //returns a value between -1.0 and 1.0
    public float RTrack() { return gamepad1.right_stick_y; }

    //if true, harvester is activated.
    public boolean HarvesterButtonPressed() { return gamepad2.right_bumper; }

    //if true, robot shoots.
    public boolean ShooterButtonPressed()   { return gamepad2.a; }

    //if true, robot presses left button.
    public boolean LeftButtonPresserButtonPressed()  { return gamepad2.dpad_left;  }

    //if true, robot presses right button.
    public boolean RightButtonPresserButtonPressed() { return gamepad2.dpad_right; }
    /**
     *
     */

}
