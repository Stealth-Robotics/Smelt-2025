package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final BeltSubsystem beltSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;
    private final Servo hatServo;
    private final Servo shootServo;
    private final Servo shootLed;
    private final MotorVelocityReader motorVelocityReader1;
    private final MotorVelocityReader motorVelocityReader2;
    private static final double TICKS_PER_REV = 28;
    private double currentRpm = 0;
    private final ElapsedTime shootTimer = new ElapsedTime();
    public static final double MAX_RPM = 4500;
    public static final double MIN_RPM = -4000;
    private double far_rpm = 3700;
    private double near_rpm = 2650;
    private double cycle_rpm = 500;
    private double reverse_rpm = -1500;
    private double far_shot_pos = 0.31;
    private double top_pos = 0;
    private double bottom_pos = 0.67;
    private double VELOCITY_TOLERANCE_LOW = 10;
    private double VELOCITY_TOLERANCE_HIGH = 100;
    private double MIN_SHOOT_TIME_MS = 500;
    private double MAX_SHOOT_TIME_MS = 5000;
    boolean isFarShot = false;
    boolean isNearShot = false;
    boolean isCycleShot = false; 
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private double setRpms = 0;
    public static final PIDFController ShooterController = new PIDFController(0.009, 0.0001, 0.0001, 0.00052);
    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.beltSubsystem = new BeltSubsystem(hardwareMap);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap);

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hatServo = hardwareMap.get(Servo.class, "hatServo");
        shootServo = hardwareMap.get(Servo.class, "shootServo");
        shootLed = hardwareMap.get(Servo.class, "shootLed");

        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize velocity readers to get filtered RPM values from the motors.
        motorVelocityReader1 = new MotorVelocityReader(shooterMotor1, TICKS_PER_REV);
        motorVelocityReader2 = new MotorVelocityReader(shooterMotor2, TICKS_PER_REV);

    }

    public void update(){
        updateAverageRpm();
    }

    public BeltSubsystem getBeltSubsystem() {
        return this.beltSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intakeSubsystem;
    }


    public boolean isShootReady(double targetRpm){
        double difference = Math.abs(currentRpm - targetRpm);
        double curMs = shootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }

//        return ShooterController.atSetPoint();

        return (difference <= VELOCITY_TOLERANCE_HIGH||curMs > MAX_SHOOT_TIME_MS);
    }
    public Command shootOneBallFar(){
        return  new InstantCommand(() -> setRpm(3700)).
                andThen(new InstantCommand(() -> hoodSetPosition(far_shot_pos)).
                andThen(new WaitUntilCommand(() -> isShootReady(3700))).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.75))));
    }

    public Command shootThreeBallsNear(){
        return  new InstantCommand(() -> setHoodDown()).
                andThen(new WaitUntilCommand(() -> isShootReady(2650)).
                andThen(moveShootServoSide(2650)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.75))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(1))).
                andThen(new WaitCommand(2000)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> setRpm(0))));
    }
    public Command shootThreeBallsFar(){
            return new InstantCommand(() -> setRpm(3700)) .
                andThen(new InstantCommand(() -> hoodSetPosition(far_shot_pos)).
                andThen(new WaitUntilCommand(() -> isShootReady(3700))).
                andThen(new InstantCommand(() -> setShootServoPosition(0)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(-0.26))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(1)))).
                andThen(new WaitCommand(3000)).
                andThen(new InstantCommand(() -> beltSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> intakeSubsystem.setPower(0))).
                andThen(new InstantCommand(() -> setRpm(0))));
    }

    public void hoodSetPosition(double position) {
        hoodServo.setPosition(position);
        hoodCurrentPosition = position;
    }
    public void blockerSetPosition(double position){
        hatServo.setPosition(position);
        blockerCurrentPosition = position;
    }
    public void setShootServoPosition(double position){
        shootServo.setPosition(position);
        shootCurrentPosition = position;
    }
    public void setLedColor(double color){
        shootLed.setPosition(color);
        ledCurrentColor = color;
    }
    public void setBlockerUp(){
        blockerSetPosition(0.41);
    }
    public void setBlockerDown(){
        blockerSetPosition(0.267);
    }
    public Command moveShootServoSide(double RPM){
        return new  WaitUntilCommand(() -> isShootReady(RPM)).andThen(
                new InstantCommand(() -> setShootServoPosition(0)).andThen(
                new InstantCommand(() -> setLedColor(0.47)))
        );
    }
    public void moveShootServoUp(){
        setShootServoPosition(0.12);
        setLedColor(0.28);
    }

    public void setHoodUp(){
        hoodSetPosition(far_shot_pos);
    }
    public void setHoodDown(){
        hoodSetPosition(bottom_pos);
    }
    private double hoodCurrentPosition = 0;
    private double blockerCurrentPosition = 0;
    private double shootCurrentPosition = 0;
    private double ledCurrentColor = 0;
    public void changePositionUp(){
        if (hoodCurrentPosition >= top_pos){
            hoodCurrentPosition -= 0.01;
        }
        else{
            hoodCurrentPosition = top_pos;
        }
        hoodServo.setPosition(hoodCurrentPosition);
    }
    public void changePositionDown(){
        if (hoodCurrentPosition <= bottom_pos){
            hoodCurrentPosition += 0.01;
        }
        else{
            hoodCurrentPosition = bottom_pos;
        }
        hoodServo.setPosition(hoodCurrentPosition);
    }


    public Command setRpmFar(){
        return new InstantCommand(() -> isFarShot =!isFarShot).andThen( new ConditionalCommand(
                new InstantCommand(() -> setHoodUp()).andThen(new InstantCommand(() -> setRpm(far_rpm))).andThen(moveShootServoSide(far_rpm)),
                new InstantCommand(() -> setRpm(1000)).andThen(new InstantCommand(() -> moveShootServoUp())),
                () -> isFarShot));

    }
    public Command setRpmNear(){
        return new InstantCommand(() -> isNearShot =!isNearShot).andThen( new ConditionalCommand(
                new InstantCommand(() -> setHoodDown()).andThen(new InstantCommand(() -> setRpm(near_rpm))).andThen(moveShootServoSide(near_rpm)),
                new InstantCommand(() -> setRpm(1000)).andThen(new InstantCommand(() -> moveShootServoUp())),
                () -> isNearShot));
    }

    public Command setRpmCycle(){
        return new InstantCommand(() -> isCycleShot =!isCycleShot).andThen( new ConditionalCommand(
                new InstantCommand(() -> setHoodUp()).andThen(new InstantCommand(() -> setRpm(cycle_rpm))).andThen(moveShootServoSide(cycle_rpm)),
                new InstantCommand(() -> setRpm(0)).andThen(new InstantCommand(() -> moveShootServoUp())),
                () -> isCycleShot));
    }
    public Command setReverseRpm(double RPM){
        return new ConditionalCommand(
                new InstantCommand(() -> setShootServoPosition(0)).andThen(new InstantCommand(() -> setRpm(reverse_rpm))),
                new InstantCommand(() -> setRpm(1000)).andThen(new InstantCommand(() -> moveShootServoUp())),
                () -> RPM == reverse_rpm);
    }
    public void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }
        setRpms = rpm;
        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        ShooterController.reset();
        ShooterController.setSetPoint(ticksPerSecond);
        resetVelocityReaders();
    }

    public double getCurrentRpm() {
        return ((((Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity())) / 2.0) / TICKS_PER_REV) * 60);
    }

    public double getAvgTicks(){
        return (shooterMotor1.getVelocity()+shooterMotor2.getVelocity())/2;
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
        double calculatedPower= ShooterController.calculate(-getAvgTicks());
        dashboardTelemetry.addData("AVG TICKS:", -getAvgTicks());
        dashboardTelemetry.addData("calculatedPower", calculatedPower);
        calculatedPower = MathUtils.clamp(calculatedPower, -1, 1);
        dashboardTelemetry.addData("setRpms", setRpms);
        dashboardTelemetry.addData(
                "CurrentRPM", getCurrentRpm());
        dashboardTelemetry.addData("isFarShot", isFarShot);
        dashboardTelemetry.update();
        telemetryM.addData("setRpms", setRpms);
        telemetryM.addData("CurrentRPM", getCurrentRpm());
        telemetryM.addData("CurrentRpmTest", currentRpm);
        telemetryM.addData("isShootReady", isShootReady(3700));
        telemetryM.addData("AverageRpm", getCurrentRpm());
        dashboardTelemetry.addData("Motor1Rpm", Math.abs(shooterMotor1.getVelocity() / TICKS_PER_REV * 60));
        dashboardTelemetry.addData("Motor2Rpm", Math.abs(shooterMotor2.getVelocity() / TICKS_PER_REV * 60));
        telemetryM.addData("hoodCurrentPos", hoodCurrentPosition);
        telemetryM.addData("hatCurrentPos", blockerCurrentPosition);
        telemetryM.addData("shootServoCurrentPos", shootCurrentPosition);
        telemetryM.addData("isFarShot", isFarShot);
        shooterMotor1.setPower(calculatedPower);
        shooterMotor2.setPower(calculatedPower);
        telemetryM.update(telemetry);
    }
}
