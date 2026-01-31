package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

public class BeltSubsystem extends StealthSubsystem {
    public double beltRatio = 1;
    private final DcMotorEx beltMotor;
    public BeltSubsystem (HardwareMap hardwareMap) {
        beltMotor = hardwareMap.get(DcMotorEx.class, "beltMotor");
    }
    public void start(){
        beltMotor.setPower(beltRatio);
    }
    public void setPower(double power){
        beltMotor.setPower(-beltRatio * power);
    }
    public void stop(){
        beltMotor.setPower(0);
    }
    public void setBeltRatio(double beltRatio){
        this.beltRatio = beltRatio;
    }
}
