package org.firstinspires.ftc.avalanche.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by austinzhang on 10/9/16.
 */
public class MotorLeftFront {
    DcMotor motor;

    public MotorLeftFront(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("LeftFront");
    }

    public DcMotor getMotor() {
        return motor;
    }

}
