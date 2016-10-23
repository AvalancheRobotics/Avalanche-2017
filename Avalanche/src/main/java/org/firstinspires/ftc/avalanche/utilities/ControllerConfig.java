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

public interface ControllerConfig {

    float LTrack();
    float RTrack();
    boolean HarvesterButtonPressed();
    boolean ShooterButtonPressed();
    boolean LeftButtonPresserButtonPressed();
    boolean RightButtonPresserButtonPressed();
    boolean modifierKey();
    boolean turnRight();
    boolean turnLeft();
    boolean increaseSpeed();
    boolean decreaseSpeed();
    boolean reverse();
    boolean forward();
}
