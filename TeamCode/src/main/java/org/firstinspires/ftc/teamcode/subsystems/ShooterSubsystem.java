package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.usablethings.MotorVelocityReader;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;
    private final MotorVelocityReader motorVelocityReader1;
    private final MotorVelocityReader motorVelocityReader2;
    private static final double TICKS_PER_REV = 28;
    private double currentRpm = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    public static final double MAX_RPM = 5200;
    public static final double MIN_RPM = 3000;
    public static final double DEFAULT_RPM_FAR = 4750;
    public static final double DEFAULT_RPM_NEAR = 4250;
    public static final double DEFFAULT_RPM_MID = 4400;
    public static final double RPM_CHANGE_AMOUNT = 50;
    private static final double VELOCITY_TOLERANCE_LOW = 50; // The allowed RPM error in which the shooter is considered "ready".
    private static final double VELOCITY_TOLERANCE_HIGH = 100;


    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(1.26, 0.126, 0, 12.9); //1.3, 0.15, 0, 12.15);
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        shooterMotor1.setPositionPIDFCoefficients(5.0);
        shooterMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        shooterMotor2.setPositionPIDFCoefficients(5.0);
        // Initialize velocity readers to get filtered RPM values from the motors.
        motorVelocityReader1 = new MotorVelocityReader(shooterMotor1, TICKS_PER_REV);
        motorVelocityReader2 = new MotorVelocityReader(shooterMotor2, TICKS_PER_REV);

    }

    public void update(){
        updateAverageRpm();
    }
    /*public void setPower(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }
    public void stop (){
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }*/
    public void setPosition(double position) {
        hoodServo.setPosition(position);
    }

    public void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm > 0 && rpm < MIN_RPM) {
            // If setting a non-zero RPM, ensure it meets the minimum operational speed
            rpm = MIN_RPM;
        }


        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        shooterMotor1.setVelocity(ticksPerSecond);
        shooterMotor2.setVelocity(ticksPerSecond);
        resetVelocityReaders();
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getMotorRpms() {
        return (((this.shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0) / TICKS_PER_REV) * 60;
    }

    private void resetVelocityReaders(){
        motorVelocityReader1.reset();
        motorVelocityReader2.reset();
        shootTimer.reset();
    }

    private void updateAverageRpm() {
        // Use the absolute value to prevent issues with one motor reporting a negative value momentarily
        double motorRpm1 = motorVelocityReader1.getFilteredRpm();
        double motorRpm2 = motorVelocityReader2.getFilteredRpm();
        this.currentRpm = (Math.abs(motorRpm1) + Math.abs(motorRpm2)) / 2.0;
    }
    public void periodic() {
        telemetry.addData("RPM", getCurrentRpm());
    }
}
