////package org.firstinspires.ftc.teamcode.Subsystems;
////
////import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
////import com.acmerobotics.roadrunner.Action;
////
////public class ArmPIDUpdater implements Action {
////
////    private final ArmPID arm;
////
////    public ArmPIDUpdater(ArmPID arm) {
////        this.arm = arm;
////    }
////
////    @Override
////    public boolean run(TelemetryPacket packet) {
////        arm.update();
////        return false; // Keeps running as part of continuous loop
////    }
////}
//
///* ----------------------------------------for bulk read thing --------------------------------------*/
//
//
//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//
//public class ArmPIDUpdater implements Action {
//
//    private final ArmPID arm;
//
//    public ArmPIDUpdater(ArmPID arm) {
//        this.arm = arm;
//    }
//
//    @Override
//    public boolean run(TelemetryPacket packet) {
//        arm.update();
//        return false; // Still just calls update once unless wrapped in a loop
//    }
//}
