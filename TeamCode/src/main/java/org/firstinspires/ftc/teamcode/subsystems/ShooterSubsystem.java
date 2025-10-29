package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void setPower(double power) {
        shooterMotor1.setPower(-power);
        shooterMotor2.setPower(-power);
    }
    public void stop (){
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
    public void setPosition(double position) {
        hoodServo.setPosition(position);
    }

}
