package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

public class BeltSubsystem extends StealthSubsystem {

    private final DcMotorEx beltMotor;
    public BeltSubsystem (HardwareMap hardwareMap) {
        beltMotor = hardwareMap.get(DcMotorEx.class, "beltMotor");
    }
    public void setPower(double power){
        beltMotor.setPower(power);
    }
}
