package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;

import java.util.function.DoubleSupplier;

public class beltCommand extends CommandBase {
    BeltSubsystem beltSubsystem;
    DoubleSupplier power;

    public beltCommand(BeltSubsystem beltSubsystem, DoubleSupplier power){
        this.beltSubsystem = beltSubsystem;
        this.power = power;

        addRequirements(beltSubsystem);
    }

    @Override
    public void execute() {
        beltSubsystem.setPower(power.getAsDouble());
    }
}
