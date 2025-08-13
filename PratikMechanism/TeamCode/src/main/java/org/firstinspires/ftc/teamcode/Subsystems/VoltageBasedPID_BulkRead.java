package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;

public class VoltageBasedPID_BulkRead {

    private BotHardware robot;

    // Voltage scaling
    private final double nominalVoltage = 13.0;

    // Nominal PID values (at full battery)
    private final double pivot_kP_nominal = 0.0009;
    private final double pivot_kI_nominal = 0.0001;
    private final double pivot_kD_nominal = 0.2;

    private final double ext_kP_nominal = 0.0001;
    private final double ext_kI_nominal = 0.0001;
    private final double ext_kD_nominal = 0.15;

    // Error terms
    private double pivotIntegral = 0;
    private double pivotLastError = 0;

    private double extIntegral = 0;
    private double extLastError = 0;

    // Control toggles
    private boolean useBulk = true;

    // Targets
    public static int pivotTarget = 0;
    public static int extensionTarget = 0;

    // Constructor
    public VoltageBasedPID_BulkRead(BotHardware robot) {
        this.robot = robot;
    }

    // Enable/disable bulk read
    public void setUseBulkRead(boolean enable) {
        this.useBulk = enable;
    }

    // Main control update
    public void update() {
        // Clear or use bulk cache if enabled
        if (useBulk && robot.expansionHub != null) {
            robot.expansionHub.clearBulkCache();
        }

        double voltage = robot.voltageSensor.getVoltage();
        double scale = Range.clip(nominalVoltage / voltage, 0.8, 1.5);

        double pivot_kP = pivot_kP_nominal * scale;
        double pivot_kI = pivot_kI_nominal * scale;
        double pivot_kD = pivot_kD_nominal * scale;

        double ext_kP = ext_kP_nominal * scale;
        double ext_kI = ext_kI_nominal * scale;
        double ext_kD = ext_kD_nominal * scale;

        // === Pivot PID ===
        double pivotPos = robot.pivot_motor.getCurrentPosition();
        double pivotError = pivotTarget - pivotPos;
        double pivotVelocity = robot.pivot_motor.getVelocity();

        if (Math.abs(pivotError) < 15 && Math.abs(pivotVelocity) < 30) {
            pivotIntegral = 0;
            pivotLastError = 0;
            robot.pivot_motor.setPower(0);
        } else {
            pivotIntegral += pivotError;
            double pivotDerivative = -pivotVelocity * 0.0015;

            double pivotPower = pivot_kP * pivotError
                    + pivot_kI * pivotIntegral
                    + pivot_kD * pivotDerivative;

            if (Math.abs(pivotPower) < 0.03) pivotPower = 0;
            robot.pivot_motor.setPower(Range.clip(pivotPower, -1.0, 1.0));
        }

        // === Extension PID ===
        double extPos = robot.extension_motor.getCurrentPosition();
        double extError = extensionTarget - extPos;
        double extVelocity = robot.extension_motor.getVelocity();

        if (Math.abs(extError) < 15 && Math.abs(extVelocity) < 30) {
            extIntegral = 0;
            extLastError = 0;
            robot.extension_motor.setPower(0);
        } else {
            extIntegral += extError;
            double extDerivative = -extVelocity * 0.002;

            double extPower = ext_kP * extError
                    + ext_kI * extIntegral
                    + ext_kD * extDerivative;

            if (Math.abs(extPower) < 0.03) extPower = 0;
            robot.extension_motor.setPower(Range.clip(extPower, -1.0, 1.0));
        }
    }

    // === Target Setters ===
    public void setPivotTarget(int target) {
        pivotTarget = target;
    }

    public void setExtensionTarget(int target) {
        extensionTarget = target;
    }

    // === Optional Enums for FSM if needed ===
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

    // === State Setters (optional for presets) ===
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
            case INIT:
                setElbow(globals.ElbowInit);
                break;
            case INTAKE:
                setElbow(globals.ElbowIntake);
                break;
            case AUTO_SAMPLE_INTAKE:
                setElbow(globals.AutoElbowIntake);
                break;
            case BASKET_DROP:
                setElbow(globals.ElbowBasketDrop);
                break;
            case SPECIMEN_PICK:
                setElbow(globals.ElbowSpecimenPick);
                break;
            case SPECIMEN_DROP:
                setElbow(globals.ElbowSpecimenDrop);
                break;
        }
    }

    // === Safe Servo Setters ===
    public void setClaw(double pos) {
        robot.Claw.setPosition(Range.clip(pos, 0.25, 0.6));
    }

    public void setWrist(double pos) {
        robot.Wrist.setPosition(Range.clip(pos, 0.2, 0.6));
    }

    public void setElbow(double pos) {
        robot.Elbow.setPosition(Range.clip(pos, 0.25, 0.96));
    }

    // === Scaled PID Getters for telemetry ===
    public double getScaledPivot_kP() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return pivot_kP_nominal * voltageScale;
    }

    public double getScaledPivot_kI() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return pivot_kI_nominal * voltageScale;
    }

    public double getScaledPivot_kD() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return pivot_kD_nominal * voltageScale;
    }

    public double getScaledExt_kP() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return ext_kP_nominal * voltageScale;
    }

    public double getScaledExt_kI() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return ext_kI_nominal * voltageScale;
    }

    public double getScaledExt_kD() {
        double voltageScale = Range.clip(nominalVoltage / robot.voltageSensor.getVoltage(), 0.8, 1.5);
        return ext_kD_nominal * voltageScale;
    }
}
