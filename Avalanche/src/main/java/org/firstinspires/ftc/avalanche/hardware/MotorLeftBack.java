package org.firstinspires.ftc.avalanche.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by austinzhang on 10/9/16.
 */
public class MotorLeftBack {
    DcMotor motor;

    public MotorLeftBack(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("LeftBack");
    }

    public DcMotor getMotor() {
        return motor;
    }

}
