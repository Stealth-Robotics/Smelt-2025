package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class BeltSubsystem extends StealthSubsystem {

    private final DcMotorEx beltMotor;
    public BeltSubsystem (HardwareMap hardwareMap) {
        beltMotor = hardwareMap.get(DcMotorEx.class, "beltMotor");
    }
    public void start(){
        beltMotor.setPower(0.75);
    }
    public void setPower(double power){
        beltMotor.setPower(0.75 * -power);
    }
    public void stop(){
        beltMotor.setPower(0);
    }
}
