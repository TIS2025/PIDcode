package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;

public class armmm {

    private final BotHardware robot;

    // PID + Feedforward gains
    public static double pivot_kP = 0.0025; //0.002; //0.0018; //0.0015; //0.003;
    public static double pivot_kD = 0.0010; //0.0008;
    public static double pivot_kG = 0.07; //0.06; //0.09;
    public static double pivot_kF = 0.001; // 0.015; //0.02; //0.05;
    public static double ext_kP = 0.004;
    public static double ext_kF = 0.0005;

    // Masses (kg)
    public static double m1 = 0.209, m2 = 0.257, m3 = 0.086, m4 = 0.058;

    // Geometry
    public static double length_r = 1.3;            // distance from pivot base to extension mount
    public static double length_last = 0.3;         // last pipe length
    private static final double dist_cog_cor_max = 1.05;
    private static final double TICKS_PER_DEGREE = 180.0 / 2400.0;
    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double TICKS_PER_METER = 1200.0;

    // Deadbands
    private static final int pivotDeadband = 20;
    private static final int extensionDeadband = 10;

    // Targets + Control Flags
    public static int pivotTarget = 0;
    public static int extensionTarget = 0;
    private boolean pivotEnabled = false;
    private boolean extensionEnabled = false;

    private int lastPivotError = 0;
    // Debug Feedforward for telemetry
    public double debugPivotFF = 0;

    // States
    public enum ClawState { INIT, OPEN, CLOSE }
    public enum WristState { INIT, WRIST0, WRIST90, SAMPLE_PICK, SAMPLE_DROP, SPECIMEN_PICK, SPECIMEN_DROP }
    public enum ElbowState { INIT, INTAKE, AUTO_SAMPLE_INTAKE, BASKET_DROP, SPECIMEN_PICK, SPECIMEN_DROP }
    public enum PivotState { INIT, SAMPLE_DROP, SAMPLE_PICK, SAMPLE_PRE_PICK }
    public enum ExtensionState { INIT, SAMPLE_DROP, SAMPLE_PICK }

    public ClawState claw = ClawState.INIT;
    public WristState wrist = WristState.INIT;
    public ElbowState elbow = ElbowState.INIT;
    public PivotState pivot = PivotState.INIT;
    public ExtensionState extension = ExtensionState.INIT;

    public static boolean isWrist0 = true;

    public armmm(BotHardware robot, Telemetry telemetry) {
        this.robot = robot;
    }

    public void update() {
        updatePivot();
        updateExtension();
    }

    private void updatePivot() {
        int pivotPos = robot.pivot_motor.getCurrentPosition();
        int errorTicks = pivotTarget - pivotPos;
        double angleRad = pivotPos * TICKS_PER_DEGREE * DEG_TO_RAD;

        int errorDelta = errorTicks - lastPivotError;
        lastPivotError = errorTicks;
        double derivative = Math.abs(errorTicks) > 40 ? pivot_kD * errorDelta : 0;

        double ff = getDynamicPivotFeedforward(angleRad, robot.extension_motor.getCurrentPosition());
        debugPivotFF = ff;

        // Deadband stop
        if (Math.abs(errorTicks) < pivotDeadband && Math.abs(errorDelta) < 2) {
            robot.pivot_motor.setPower(0);
            pivotEnabled = false;
            return;
        }

        double control = pivot_kP * errorTicks + derivative + ff;

        if (Math.abs(Math.cos(angleRad)) < 0.25 && Math.abs(errorTicks) < 60 && control < 0.08) {
            control = 0.08;  // Helps hold horizontal weight without jitter
        }

        robot.pivot_motor.setPower(control);
    }


    private void updateExtension() {
        int extPos = robot.extension_motor.getCurrentPosition();
        int error = extensionTarget - extPos;

        if (Math.abs(error) < extensionDeadband) {
            extensionEnabled = false;
            robot.extension_motor.setPower(0);
            return;
        }

        if (!extensionEnabled) return;

        double power = ext_kP * error + ext_kF;
        robot.extension_motor.setPower(Range.clip(power, -1.0, 1.0));
    }


    private double getDynamicPivotFeedforward(double angleRad, int extensionTicks) {
        double extension_m = extensionTicks / TICKS_PER_METER;

        double cog = m2 * extension_m + m3 * (extension_m + 0.3) + m4 * (extension_m + 0.6);
        cog /= (m1 + m2 + m3 + m4);

        double dist_cog_cor = Math.sqrt(cog * cog + length_r * length_r);
        double dist_ratio = dist_cog_cor / dist_cog_cor_max;

        double rawFF = pivot_kG * Math.cos(-angleRad) * dist_ratio;

        // Apply minimum FF vertically
        if (Math.abs(Math.sin(angleRad)) > 0.98 && rawFF < 0.04) {
            rawFF = 0.04;
        }

        // Clamp and enforce FF at horizontal to prevent sag
        if (Math.abs(Math.cos(angleRad)) < 0.2) {
            rawFF = 0.05;
        } else if (rawFF < 0) {
            rawFF = 0;
        }

        return rawFF + pivot_kF + 0.01;  // small static boost
    }

    // ========== Setters ==========

    public void setPivotTarget(int target) {
        pivotTarget = target;
        pivotEnabled = true;
    }

    public void setExtensionTarget(int target) {
        extensionTarget = target;
        extensionEnabled = true;
    }

    public void updatePivotState(@NonNull PivotState state) {
        this.pivot = state;
        switch (state) {
            case INIT: setPivotTarget(globals.PivotInit); break;
            case SAMPLE_PRE_PICK: setPivotTarget(globals.PivotPreSamplePick); break;
            case SAMPLE_PICK: setPivotTarget(globals.PivotSamplePick); break;
            case SAMPLE_DROP: setPivotTarget(globals.PivotSampleDrop); break;
        }
    }

    public void updateExtensionState(@NonNull ExtensionState state) {
        this.extension = state;
        switch (state) {
            case INIT: setExtensionTarget(globals.ExtensionInit); break;
            case SAMPLE_PICK: setExtensionTarget(globals.ExtensionSamplePick); break;
            case SAMPLE_DROP: setExtensionTarget(globals.ExtensionSampleDrop); break;
        }
    }

    public void updateClawState(@NonNull ClawState state) {
        this.claw = state;
        switch (state) {
            case INIT: setClaw(globals.ClawInit); break;
            case OPEN: setClaw(globals.ClawOpen); break;
            case CLOSE: setClaw(globals.ClawClose); break;
        }
    }

    public void updateWristState(@NonNull WristState state) {
        this.wrist = state;
        switch (state) {
            case INIT: setWrist(globals.WristInit); break;
            case WRIST0:setWrist(globals.Wrist0); break;
            case WRIST90: setWrist(globals.Wrist90); break;
            case SAMPLE_PICK: setWrist(globals.WristSamplePick); break;
            case SAMPLE_DROP: setWrist(globals.WristSampleDrop); break;
            case SPECIMEN_PICK: setWrist(globals.WristSpecimenPick); break;
            case SPECIMEN_DROP: setWrist(globals.WristSpecimenDrop); break;
        }
    }

    public void updateElbowState(@NonNull ElbowState state) {
        this.elbow = state;
        switch (state) {
            case INIT: setElbow(globals.ElbowInit); break;
            case INTAKE: setElbow(globals.ElbowIntake); break;
            case BASKET_DROP: setElbow(globals.ElbowBasketDrop); break;
            case AUTO_SAMPLE_INTAKE: setElbow(globals.AutoElbowIntake); break;
            case SPECIMEN_PICK: setElbow(globals.ElbowSpecimenPick); break;
            case SPECIMEN_DROP: setElbow(globals.ElbowSpecimenDrop); break;
        }
    }

    // ========== Safe Servo Setters ==========

    public void setClaw(double pos) {
        robot.Claw.setPosition(Range.clip(pos, 0.25, 0.6));
    }

    public void setWrist(double pos) {
        robot.Wrist.setPosition(Range.clip(pos, 0.2, 0.6));
    }

    public void setElbow(double pos) {
        robot.Elbow.setPosition(Range.clip(pos, 0.25, 0.96));
    }
}
