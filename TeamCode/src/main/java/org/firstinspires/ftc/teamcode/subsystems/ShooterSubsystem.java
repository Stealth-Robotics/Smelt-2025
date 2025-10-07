package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }

    private void setPower(double power) {
        shooterMotor.setPower(power);
    }
}
