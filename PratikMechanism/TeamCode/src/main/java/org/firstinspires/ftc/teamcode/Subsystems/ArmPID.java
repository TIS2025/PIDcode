/*---------------------------------------------------- Simple PID ------------------------------------------------------------*/

//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
//
//public class ArmPID {
//
//    private final BotHardware robot;
//
//    // PID Constants
//    public static double pivot_kP = 0.009; //0.01;  //0.005;
//    public static double pivot_kI = 0.0;
//    public static double pivot_kD = 0.002; //0.001; //0.0; //0.15;
//
//    public static double ext_kP = 0.01; //0.009;
//    public static double ext_kI = 0.0;
//    public static double ext_kD = 0.009; //0.007; //0.001; //0.0; //0.05;
//
//    // Integral and last error storage
//    private double pivotIntegral = 0;
//    private double pivotLastError = 0;
//
//    private double extIntegral = 0;
//    private double extLastError = 0;
//
//    // Target positions
//    public static int pivotTarget = 0;
//    public static int extensionTarget = 0;
//
//    // Constructor
//    public ArmPID(BotHardware robot) {
//        this.robot = robot;
//    }
//
//    // Update method to be called in TeleOp loop
//    public void update() {
//        // === PIVOT PID ===
//        double pivotPos = robot.pivot_motor.getCurrentPosition();
//        double pivotError = pivotTarget - pivotPos;
//
//        if (Math.abs(pivotError) < 10) {
//            pivotIntegral = 0;
//            robot.pivot_motor.setPower(0);
//        } else {
//            pivotIntegral += pivotError;
//            double pivotDerivative = pivotError - pivotLastError;
//            pivotLastError = pivotError;
//
//            double pivotPower = pivot_kP * pivotError + pivot_kI * pivotIntegral + pivot_kD * pivotDerivative;
//            pivotPower = Range.clip(pivotPower, -1.0, 1.0);
//
//            robot.pivot_motor.setPower(pivotPower);
//        }
//
//        // === EXTENSION PID ===
//        double extPos = robot.extension_motor.getCurrentPosition();
//        double extError = extensionTarget - extPos;
//
//        if (Math.abs(extError) < 10) {
//            extIntegral = 0;
//            robot.extension_motor.setPower(0);
//        } else {
//            extIntegral += extError;
//            double extDerivative = extError - extLastError;
//            extLastError = extError;
//
//            double extPower = ext_kP * extError + ext_kI * extIntegral + ext_kD * extDerivative;
//            extPower = Range.clip(extPower, -1.0, 1.0);
//
//            robot.extension_motor.setPower(extPower);
//        }
//    }
//
//
//    public void setPivotTarget(int target) {
//        pivotTarget = target;
//    }
//
//    public void setExtensionTarget(int target) {
//        extensionTarget = target;
//    }
//}
