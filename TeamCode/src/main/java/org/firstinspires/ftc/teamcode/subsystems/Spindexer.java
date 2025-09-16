package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class Spindexer extends StealthSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final CRServo servo1;
    private final CRServo servo2;

    private int currentIndex = 0;

    //TODO: Figure Out Starting Configuration
    private final Artifact[] spindexer = {Artifact.GREEN, Artifact.PURPLE, Artifact.GREEN};

    private boolean isMoving = false;

    public Spindexer(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        servo1 = hardwareMap.get(CRServo.class, "spinServo1");
        servo2 = hardwareMap.get(CRServo.class, "spinServo2");
    }

    public Command rotateToSlot(int desiredIndex) {
        boolean shortestIsLeft = ((currentIndex - desiredIndex) % 3 < (currentIndex + desiredIndex) % 3);

        return new ConditionalCommand(
                rotateToTheLeft(desiredIndex),
                rotateToTheRight(desiredIndex),
                () -> shortestIsLeft).andThen(new InstantCommand(() -> currentIndex = desiredIndex)
        );
    }

    //TODO: Set Servo Directions
    private Command rotateToTheLeft(int index) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(-1)),
                        new InstantCommand(() -> servo2.setPower(-1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    //TODO: Set Servo Directions
    private Command rotateToTheRight(int index) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(1)),
                        new InstantCommand(() -> servo2.setPower(1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    //Use color + distance color sensor measurement to accurately detect when a slot if full
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
