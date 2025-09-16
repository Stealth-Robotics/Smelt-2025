package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;
import static java.lang.Math.abs;

public class SpindexerSubsystem extends StealthSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final CRServo servo1;
    private final CRServo servo2;

    private int currentIndex = 0;

    private final Artifact[] spindexer = {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};

    private boolean isMoving = false;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        servo1 = hardwareMap.get(CRServo.class, "spinServo1");
        servo2 = hardwareMap.get(CRServo.class, "spinServo2");
    }

    public void rotateToSlot(int desiredIndex) {
        boolean shortestIsLeft = ((currentIndex - desiredIndex) % 3 < (currentIndex + desiredIndex) % 3);
        int rotations = abs(currentIndex - desiredIndex);

        currentIndex = desiredIndex;

        if (shortestIsLeft)
            for (int i = rotations; i > 0; i--)
                rotateLeft();
        else
            for (int i = rotations; i > 0; i--)
                rotateRight();
    }

    private Command rotateLeft() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(-1)),
                        new InstantCommand(() -> servo2.setPower(-1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    private Command rotateRight() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(1)),
                        new InstantCommand(() -> servo2.setPower(1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    //Use color + distance color sensor measurement to accurately detect when a slot is full
    private boolean detectedGamePiece() {
        return true;
    }

    public int getCurrentSlot() {
        return currentIndex;
    }

    public void setSlotColor(Artifact artifact) {
        spindexer[currentIndex] = artifact;
    }

    @Override
    public void periodic() {
        telemetry.addData("Slot: ", currentIndex);
        telemetry.addData("Moving: ", isMoving);
    }
}
