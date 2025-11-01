package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.usablethings.MotorVelocityReader;
import org.stealthrobotics.library.StealthSubsystem;

@Config
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;
    private final MotorVelocityReader motorVelocityReader1;
    private final MotorVelocityReader motorVelocityReader2;
    private static final double TICKS_PER_REV = 28;
    private double currentRpm = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    public static final double MAX_RPM = 4500;
    public static final double MIN_RPM = 3000;
    public static final double DEFAULT_RPM_FAR = 4750;
    public static final double DEFAULT_RPM_NEAR = 4250;
    public static final double DEFAULT_RPM_MID = 4400;
    public static final double RPM_CHANGE_AMOUNT = 50;
    private static final double VELOCITY_TOLERANCE_LOW = 50; // The allowed RPM error in which the shooter is considered "ready".
    private static final double VELOCITY_TOLERANCE_HIGH = 100;
    private double top_pos = 0.8;
    private double bottom_pos = 0.3;

    public static final double KP = 1;//old: -4.7
    public static final double KI = 0;//old: 0
    public static final double KD = 2;//old: 2
    public PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(KP, KI, KD, 17);
    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        //shooterMotor1.setPositionPIDFCoefficients(5.0);
        shooterMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//        shooterMotor2.setPositionPIDFCoefficients(5.0);
        // Initialize velocity readers to get filtered RPM values from the motors.
        motorVelocityReader1 = new MotorVelocityReader(shooterMotor1, TICKS_PER_REV);
        motorVelocityReader2 = new MotorVelocityReader(shooterMotor2, TICKS_PER_REV);

    }

    public void update(){
        updateAverageRpm();
    }

    public void setPosition(double position) {
        hoodServo.setPosition(position);
        currentPosition = position;
    }
    private double currentPosition = 0;
    public void changePositionUp(){
        if (currentPosition <= top_pos){
            currentPosition += 0.01;
        }
        else{
            currentPosition = top_pos;
        }
        hoodServo.setPosition(currentPosition);
    }
    public void changePositionDown(){
        if (currentPosition >= bottom_pos){
            currentPosition -= 0.01;
        }
        else{
            currentPosition = bottom_pos;
        }
        hoodServo.setPosition(currentPosition);
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
    @Override
    public void periodic() {
        update();

        telemetry.addData("RPM", getCurrentRpm());
        telemetry.addData("currentPos", currentPosition);
    }
}
