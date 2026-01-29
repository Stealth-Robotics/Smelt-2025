package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.MathFunctions;
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
    // ! Disable in matches to reduce latency
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

    public static final PIDFController shooterController = new PIDFController(0.009, 0.0001, 0.0001, 0.00052);

    private static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 4500;
    public static final double MIN_RPM = -4000;
    private double VELOCITY_TOLERANCE = 100;

    private double currentRPM = 0;
    private double targetRPM = 0;

    private final Command shootWaitCommand = new SequentialCommandGroup(new WaitUntilCommand(this::isReadyToShoot), allowShooting());

    public enum ShooterMode {
        FAR_SHOT(3700),
        NEAR_SHOT(2650),
        IDLE(1000),
        CYCLE(500),
        REVERSE(-1500),
        STOPPED(0);

        public int rpm;
        ShooterMode(int rpm) {
            this.rpm = rpm;
        }
    }

    public enum HoodMode {
        MAX(0),
        FAR_SHOT(0.26),
        NEAR_SHOT(0.71);

        public double angle;
        HoodMode(double angle) {
            this.angle = angle;
        }
    }

    public ShooterSubsystem(HardwareMap hardwareMap, BeltSubsystem beltSubsystem, IntakeSubsystem intakeSubsystem) {
        this.beltSubsystem = beltSubsystem;
        this.intakeSubsystem = intakeSubsystem;

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

    public boolean isReadyToShoot() {
        return (Math.abs(currentRPM - targetRPM) <= VELOCITY_TOLERANCE);
    }

    public Command shootThreeBallsNear() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setHoodPosition(HoodMode.NEAR_SHOT)),
                shootWhenReady(),
                new InstantCommand(() -> beltSubsystem.setPower(-0.75)),
                new InstantCommand(() -> intakeSubsystem.setPower(1)),
                new WaitCommand(2000),
                new InstantCommand(() -> beltSubsystem.setPower(0)),
                new InstantCommand(() -> intakeSubsystem.setPower(0)),
                idle() // Idle the shooter
        );
    }

    public Command shootThreeBallsFar() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setHoodPosition(HoodMode.FAR_SHOT)),
                shootWhenReady(),
                new InstantCommand(() -> beltSubsystem.setPower(-0.26)),
                new InstantCommand(() -> intakeSubsystem.setPower(1)),
                new WaitCommand(3000),
                new InstantCommand(() -> beltSubsystem.setPower(0)),
                new InstantCommand(() -> intakeSubsystem.setPower(0)),
                idle() // Idle the shooter
        );
    }

    public void setHoodPosition(HoodMode mode) {
        hoodServo.setPosition(mode.angle);
    }

    private void setBlockerPosition(double position) {
        hatServo.setPosition(position);
    }

    public void setBlockerUp() {
        setBlockerPosition(0.41);
    }

    public void setBlockerDown() {
        setBlockerPosition(0.267);
    }

    public void setShootServoPosition(double position) {
        shootServo.setPosition(position);
    }

    public void setLedColor(double color) {
        shootLed.setPosition(color);
    }

    // Waits until the shooter is at its target RPM and then allows balls to pass into the shooter
    public Command shootWhenReady() {
        return shootWaitCommand;
    }

    //Opens the latch that blocks artifacts from entering the shooter
    public Command allowShooting() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setShootServoPosition(0)),
                new InstantCommand(() -> setLedColor(0.47))
        );
    }

    //Closes the latch that blocks artifacts from entering the shooter
    public Command preventShooting() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setShootServoPosition(0.12)),
                new InstantCommand(() -> setLedColor(0.28))
        );
    }

    // SHOOT COMMANDS
    public Command shootFar() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setHoodPosition(HoodMode.FAR_SHOT)),
                new InstantCommand(() -> setShooterRpm(ShooterMode.FAR_SHOT)),
                shootWhenReady()
        );
    }

    public Command shootNear() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setHoodPosition(HoodMode.NEAR_SHOT)),
                new InstantCommand(() -> setShooterRpm(ShooterMode.NEAR_SHOT)),
                shootWhenReady()
        );
    }

    //Set the shooter to run at its idle RPMs and reset the shot blocking latch
    public Command idle() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setShooterRpm(ShooterMode.IDLE)),
                preventShooting()
        );
    }

    //Set the shooter to start cycling the artifacts to rearrange their order in the indexer
    public Command cycle() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setHoodPosition(HoodMode.FAR_SHOT)),
                new InstantCommand(() -> setShooterRpm(ShooterMode.CYCLE)),
                shootWhenReady()
        );
    }

    //Reverse the shooter
    public Command reverse() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setShootServoPosition(0)),
                new InstantCommand(() -> setShooterRpm(ShooterMode.REVERSE))
        );
    }

    //Command that cancels all ongoing shooter actions and idles
    public Command forceIdle() {
        return new SequentialCommandGroup(
                new InstantCommand(shootWaitCommand::cancel),
                idle()
        );
    }

    public void setShooterRpm(ShooterMode mode) {
        targetRPM = MathFunctions.clamp(mode.rpm, MIN_RPM, MAX_RPM);

        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = targetRPM * TICKS_PER_REV / 60;
        shooterController.reset();
        shooterController.setSetPoint(ticksPerSecond);
    }

    public double getCurrentRpm() {
        return ((((Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity())) / 2.0) / TICKS_PER_REV) * 60);
    }

    public double getAvgTicks() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2;
    }

    public void resetVelocityReaders() {
        motorVelocityReader1.reset();
        motorVelocityReader2.reset();
    }

    private void updateRPMEstimate() {
        // Use the absolute value to prevent issues with one motor reporting a negative value momentarily
        double motorRpm1 = motorVelocityReader1.getFilteredRpm();
        double motorRpm2 = motorVelocityReader2.getFilteredRpm();
        currentRPM = (Math.abs(motorRpm1) + Math.abs(motorRpm2)) / 2.0;
    }

    @Override
    public void periodic() {
        double calculatedPower = shooterController.calculate(-getAvgTicks());
        shooterMotor1.setPower(calculatedPower);
        shooterMotor2.setPower(calculatedPower);

        updateRPMEstimate();

        // TELEMETRY HANDLED BELOW HERE

        telemetry.addData("calculatedPower", calculatedPower);
        telemetry.addData("CurrentRPM", getCurrentRpm());
        telemetry.addData("Motor1Rpm", Math.abs(shooterMotor1.getVelocity() / TICKS_PER_REV * 60));
        telemetry.addData("Motor2Rpm", Math.abs(shooterMotor2.getVelocity() / TICKS_PER_REV * 60));

        // ! Disable in matches to reduce latency
        telemetryM.addData("CurrentRPM", getCurrentRpm());
        telemetryM.addData("isReadyToShoot", isReadyToShoot());
        telemetryM.addData("AverageRpm", getCurrentRpm());
        telemetryM.update(telemetry);
    }
}
