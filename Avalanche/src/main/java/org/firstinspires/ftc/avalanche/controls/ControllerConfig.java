package org.firstinspires.ftc.avalanche.controls;

/**
 * An interface that allows binding of different keys to different actions.
 *
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
    boolean LoadAndShootButtonPressed();
    boolean AutoLeftButtonPresserButtonPressed();
    boolean AutoRightButtonPresserButtonPressed();
    boolean modifierKey();
    boolean turnRight();
    boolean turnLeft();
    boolean increaseSpeed();
    boolean decreaseSpeed();
    boolean reverse();
    boolean forward();
    boolean ReverseHarvesterButtonPressed();
    boolean LockPositionButtonPressed();
    boolean ShooterActivationButtonPressed();
    float ShootOneBall();
    boolean ExtendButtonPresser();
    boolean RetractButtonPresser();
    boolean TiltButtonPresserLeftUp();
    boolean TiltButtonPresserRightUp();
    boolean HoldButtonPresserPosition();
}
