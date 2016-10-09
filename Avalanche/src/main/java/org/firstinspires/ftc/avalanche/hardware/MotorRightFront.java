package org.firstinspires.ftc.avalanche.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by austinzhang on 10/9/16.
 */
public class MotorRightFront {
    DcMotor motor;

    public MotorRightFront(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("RightFront");
    }

    public DcMotor getMotor() {
        return motor;
    }

}
