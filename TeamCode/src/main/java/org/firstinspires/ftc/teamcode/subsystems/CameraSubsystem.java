package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

public class CameraSubsystem extends StealthSubsystem {
        private final Limelight3A limelight;
        private final TelemetryManager telemetryM;
        private final LLResult lastResult;
        public CameraSubsystem (HardwareMap hardwareMap, LLResult lastResult){
            this.lastResult = lastResult;
            this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
        }
}
