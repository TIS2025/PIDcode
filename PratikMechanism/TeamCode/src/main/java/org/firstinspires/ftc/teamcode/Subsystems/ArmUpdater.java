package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ArmUpdater implements Action {

    private final armmm arm;

    public ArmUpdater(armmm arm) {
        this.arm = arm;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        arm.update();
        return false; // Keep running until the sequence it's inside finishes
    }
}
