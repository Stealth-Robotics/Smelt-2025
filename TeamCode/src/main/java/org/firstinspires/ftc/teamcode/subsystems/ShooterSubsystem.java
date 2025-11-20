package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
    private final BeltSubsystem beltSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;
    private final MotorVelocityReader motorVelocityReader1;
    private final MotorVelocityReader motorVelocityReader2;
    private static final double TICKS_PER_REV = 28;
    private double currentRpm = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    public static final double MAX_RPM = 4500;
    public static final double MIN_RPM = -4000;
    private double far_shot_pos = 0.24;
    private double top_pos = 0;
    private double bottom_pos = 0.65;
    private double VELOCITY_TOLERANCE_LOW = 10;
    private double VELOCITY_TOLERANCE_HIGH = 100;
    private double MIN_SHOOT_TIME_MS = 500;
    private double MAX_SHOOT_TIME_MS = 5000;
    public static final double KP = 40;//old: 40
    public static final double KI = 0;//old: 0.02
    public static final double KD = 0.7;//old: 0.7

    public PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(KP, KI, KD, 1);
    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.beltSubsystem = new BeltSubsystem(hardwareMap);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap);

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);

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
    public boolean isShootReady(double targetRpm){
        double difference = Math.abs(currentRpm - targetRpm);
        double curMs = shootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }
        return (difference <= VELOCITY_TOLERANCE_HIGH||curMs > MAX_SHOOT_TIME_MS);
    }
    public Command shootOneBallFar(){
        return  new InstantCommand(() -> setRpm(3500)).
                andThen(new InstantCommand(() -> setPosition(far_shot_pos)).
                andThen(new WaitUntilCommand(() -> isShootReady(3500))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.75))));
    }
    public Command shootThreeBallsNear(){
        return //InstantCommand(() -> beltSubsystem.setPower(0.5)).
//                andThen(new InstantCommand(() -> setRpm(-100)).
//                andThen(new WaitCommand(1000)).
                //andThen(new InstantCommand(() -> setRpm(2500))).
                new InstantCommand(() -> setHoodDown()).
                andThen(new WaitUntilCommand(() -> isShootReady(2550))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.4))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(1))).
                andThen(new WaitCommand(2000)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> setRpm(-1000))).
                andThen(new WaitCommand(500)).
                andThen(new InstantCommand(() -> setRpm(0)));
    }
    public Command shootThreeBallsFar(){
        return new InstantCommand(() -> beltSubsystem.setPower(0.5)).
                andThen(new WaitCommand(600)).
                andThen(new InstantCommand(() -> setRpm(3500))).
                andThen(new InstantCommand(() -> setPosition(far_shot_pos))).
                andThen(new WaitUntilCommand(() -> isShootReady(3500))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.5))).
                andThen(new WaitCommand(100)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0.25))).
                andThen(new WaitUntilCommand(() -> isShootReady(3500))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.5))).
                andThen(new WaitCommand(100)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0.25))).
                andThen(new WaitUntilCommand(() -> isShootReady(3500))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.5))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(1))).
                andThen(new WaitCommand(2000)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> setRpm(-100))).
                andThen(new WaitCommand(200)).
                andThen(new InstantCommand(() -> setRpm(0)));
    }

    public void setPosition(double position) {
        hoodServo.setPosition(position);
        currentPosition = position;
    }
    public void setHoodUp(){
        setPosition(top_pos);
    }
    public void setHoodDown(){
        setPosition(bottom_pos);
    }
    private double currentPosition = 0;
    public void changePositionUp(){
        if (currentPosition >= top_pos){
            currentPosition -= 0.01;
        }
        else{
            currentPosition = top_pos;
        }
        hoodServo.setPosition(currentPosition);
    }
    public void changePositionDown(){
        if (currentPosition <= bottom_pos){
            currentPosition += 0.01;
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
        telemetry.addData("isShootReady", isShootReady(3500));
        telemetry.addData("RPM", getCurrentRpm());
        telemetry.addData("currentPos", currentPosition);
    }
}
