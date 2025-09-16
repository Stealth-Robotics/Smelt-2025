package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo pusherServo;
    private final double CLOSED_PUSHER = 0;
    private final double OPEN_PUSHER = 1;

    public ShooterSubsystem(HardwareMap hardwareMap){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
    }

    public void setPower(double power){
        shooterMotor.setPower(power);
    }

    public void setPusher(double pos){
        pusherServo.setPosition(pos);
    }
}
