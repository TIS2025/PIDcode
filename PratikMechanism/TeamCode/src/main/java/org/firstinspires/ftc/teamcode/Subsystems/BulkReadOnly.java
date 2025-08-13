package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;

public class BulkReadOnly {

    private BotHardware robot;

    // PID constants
    public static double pivot_kP = 0.009;
    public static double pivot_kI = 0.0;
    public static double pivot_kD = 0.001;
    public static double ext_kP = 0.09; //0.02,
    public static double ext_kI = 0.0;
    public static double ext_kD = 0.0005;

    public static int pivotTarget = 0;
    public static int extensionTarget = 0;

    private double pivotErrorSum = 0, pivotLastError = 0;
    private double extErrorSum = 0, extLastError = 0;

    // Telemetry
    public double loopTimeMs = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Toggle for only clearing bulk cache (no read)
    private boolean useBulkClear = true;

    // Enums for states
    public enum ClawState { INIT, OPEN, CLOSE }
    public enum WristState { INIT, WRIST0, WRIST90, SAMPLE_PICK, SAMPLE_DROP, SPECIMEN_PICK, SPECIMEN_DROP }
    public enum ElbowState { INIT, INTAKE, AUTO_SAMPLE_INTAKE, BASKET_DROP, SPECIMEN_PICK, SPECIMEN_DROP }
    public enum PivotState { INIT, SAMPLE_DROP, SAMPLE_PICK, SAMPLE_PRE_PICK }
    public enum ExtensionState { INIT, SAMPLE_DROP, SAMPLE_PICK }

    // Current States
    public ClawState claw = ClawState.INIT;
    public WristState wrist = WristState.INIT;
    public ElbowState elbow = ElbowState.INIT;
    public PivotState pivot = PivotState.INIT;
    public ExtensionState extension = ExtensionState.INIT;

    public static boolean isWrist0 = true;

    // Constructor
    public BulkReadOnly(BotHardware robot) {
        this.robot = robot;
    }

    public void setUseBulkClear(boolean useBulk) {
        this.useBulkClear = useBulk;
    }

    public void update() {
        loopTimer.reset();

        if (useBulkClear && robot.expansionHub != null) {
            robot.expansionHub.clearBulkCache();  // Clear cache only, don't read
        }

        int pivotPos = robot.pivot_motor.getCurrentPosition();
        int extPos = robot.extension_motor.getCurrentPosition();

        // Pivot PID
        int pivotError = pivotTarget - pivotPos;
        pivotErrorSum += pivotError;
        double pivotDerivative = pivotError - pivotLastError;

        double pivotPower = pivot_kP * pivotError + pivot_kI * pivotErrorSum + pivot_kD * pivotDerivative;
        robot.pivot_motor.setPower(Range.clip(pivotPower, -1.0, 1.0));
        pivotLastError = pivotError;

        // Extension PID
        int extError = extensionTarget - extPos;
        extErrorSum += extError;
        double extDerivative = extError - extLastError;

        double extPower = ext_kP * extError + ext_kI * extErrorSum + ext_kD * extDerivative;
        robot.extension_motor.setPower(Range.clip(extPower, -1.0, 1.0));
        extLastError = extError;

        loopTimeMs = loopTimer.milliseconds();
    }

    // Target setters
    public void setPivotTarget(int target) { pivotTarget = target; }
    public void setExtensionTarget(int target) { extensionTarget = target; }

    // =================== State Setters ===================

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
            case WRIST0: setWrist(globals.Wrist0); break;
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
            case AUTO_SAMPLE_INTAKE: setElbow(globals.AutoElbowIntake); break;
            case BASKET_DROP: setElbow(globals.ElbowBasketDrop); break;
            case SPECIMEN_PICK: setElbow(globals.ElbowSpecimenPick); break;
            case SPECIMEN_DROP: setElbow(globals.ElbowSpecimenDrop); break;
        }
    }

    // Servo setters
    public void setClaw(double pos) {
        robot.Claw.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void setWrist(double pos) {
        robot.Wrist.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    public void setElbow(double pos) {
        robot.Elbow.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    // PID constant getters
    public double getScaledPivot_kP() { return pivot_kP; }
    public double getScaledPivot_kI() { return pivot_kI; }
    public double getScaledPivot_kD() { return pivot_kD; }

    public double getScaledExt_kP() { return ext_kP; }
    public double getScaledExt_kI() { return ext_kI; }
    public double getScaledExt_kD() { return ext_kD; }

    public double getLoopTimeMs() {
        return loopTimeMs;
    }
}
