package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;

public class beltCommand extends CommandBase {
    public beltCommand(BeltSubsystem belt){
            belt.setPower(1);
    }
}
