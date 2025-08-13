//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.TeleOp.PGFteleop.ftc;
//import static org.firstinspires.ftc.teamcode.TeleOp.PGFteleop.updateAction;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Globals.globals;
//import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystems.ArmPID;
//import org.firstinspires.ftc.teamcode.Subsystems.ArmPIDUpdater;
//import org.firstinspires.ftc.teamcode.Subsystems.BulkReadOnly;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Config
//@TeleOp(name = "PID Bulk Mode Compare", group = "Teleop")
//public class PIDwithBulkRead extends LinearOpMode {
//
//    public static BotHardware robot = BotHardware.getInstance();
//    private BulkReadOnly armmmm;
//    private MecanumDrive drive;
//    public static boolean flipped = true;
//    public static double strafe = 1, speed = 1, turn = 1;
//
//    // Toggle bulk read & loop telemetry
//    private boolean useBulk = true;
//    private boolean showLoop = true;
//
//    private final double TARGET_BULK_LOOP = 8.0;
//    private final double TARGET_NO_BULK_LOOP = 12.0;
//    private final double TARGET_BULK_NO_LOOP = 8.0;
//    private final double TARGET_NO_BULK_NO_LOOP = 12.0;
//
//
//    private enum BulkMode {
//        BULK_LOOP,      // gamepad2.a
//        NO_BULK_LOOP,   // gamepad2.b
//        BULK_NO_LOOP,   // gamepad2.x
//        NO_BULK_NO_LOOP // gamepad2.y
//    }
//
//    private BulkMode currentMode = BulkMode.BULK_LOOP;
//
//    private double getTargetTimeMs(BulkMode mode) {
//        switch (mode) {
//            case BULK_LOOP: return TARGET_BULK_LOOP;
//            case NO_BULK_LOOP: return TARGET_NO_BULK_LOOP;
//            case BULK_NO_LOOP: return TARGET_BULK_NO_LOOP;
//            case NO_BULK_NO_LOOP: return TARGET_NO_BULK_NO_LOOP;
//            default: return 10.0;
//        }
//    }
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap, telemetry);
//        arm = new ArmPID(robot);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        Gamepad CG1 = new Gamepad(), CG2 = new Gamepad();
//        Gamepad PG1 = new Gamepad(), PG2 = new Gamepad();
//
//        while (opModeInInit()) {
//            armmmm.setUseBulkClear(true);
//            armmmm.updatePivotState(BulkReadOnly.PivotState.INIT);
//            armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT);
//            armmmm.updateElbowState(BulkReadOnly.ElbowState.INIT);
//            armmmm.updateClawState(BulkReadOnly.ClawState.INIT);
//            armmmm.updateWristState(BulkReadOnly.WristState.INIT);
//            armmmm.update();
//
//            telemetry.addData("Pivot", robot.pivot_motor.getCurrentPosition());
//            telemetry.addData("Extension", robot.extension_motor.getCurrentPosition());
//            telemetry.update();
//        }
//
//        waitForStart();
//        robot.resetencoder();
//
//        while (opModeIsActive()) {
//            long startTime = System.nanoTime();
//
//            PG1.copy(CG1);
//            PG2.copy(CG2);
//            CG1.copy(gamepad1);
//            CG2.copy(gamepad2);
//
//            // Toggle modes
//            if (gamepad2.a) {
//                useBulk = true;
//                showLoop = true;
//                currentMode = BulkMode.BULK_LOOP;
//            } else if (gamepad2.b) {
//                useBulk = false;
//                showLoop = true;
//                currentMode = BulkMode.NO_BULK_LOOP;
//            } else if (gamepad2.x) {
//                useBulk = true;
//                showLoop = false;
//                currentMode = BulkMode.BULK_NO_LOOP;
//            } else if (gamepad2.y) {
//                useBulk = false;
//                showLoop = false;
//                currentMode = BulkMode.NO_BULK_NO_LOOP;
//            }
//
//            armmmm.setUseBulkClear(useBulk);
//            armmmm.update();
//
//            ftc = updateAction();
//            ftc.add(new ArmPIDUpdater(armmmm));
//
//            // Drive
//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x),
//                            gamepad1.right_stick_x / 2)
//            );
//            if (gamepad1.left_trigger > 0.3) {
//                drive.setDrivePowers(
//                        new PoseVelocity2d(
//                                new Vector2d(gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2),
//                                gamepad1.right_stick_x / 3)
//                );
//            }
//
//            // === Arm Presets ===
//            if (gamepad1.right_bumper) {
//                ftc.add(new ParallelAction(
//                        new ArmPIDUpdater(armmmm),
//                        new SequentialAction(
//                                new InstantAction(() -> armmmm.setPivotTarget(-150)),
//                                new SleepAction(0.5),
//                                new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.SAMPLE_PICK)),
//                                new SleepAction(1),
//                                new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INTAKE)),
//                                new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_PICK)),
//                                new InstantAction(() -> armmmm.updateClawState(BulkReadOnly.ClawState.OPEN)),
//                                new SleepAction(0.1)
//                        )
//                ));
//            }
//
//            if (gamepad1.b) {
//                ftc.add(new ParallelAction(
//                        new ArmPIDUpdater(armmmm),
//                        new SequentialAction(
//                                new InstantAction(() -> armmmm.updatePivotState(BulkReadOnly.PivotState.SAMPLE_PICK)),
//                                new SleepAction(0.5),
//                                new InstantAction(() -> armmmm.updateClawState(BulkReadOnly.ClawState.CLOSE)),
//                                new SleepAction(1),
//                                new InstantAction(() -> armmmm.updatePivotState(BulkReadOnly.PivotState.SAMPLE_PRE_PICK)),
//                                new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_DROP)),
//                                new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INIT)),
//                                new SleepAction(0.5),
//                                new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT))
//                        )
//                ));
//            }
//
//            if (gamepad1.a) {
//                ftc.add(new ParallelAction(
//                        new ArmPIDUpdater(armmmm),
//                        new SequentialAction(
//                                new InstantAction(() -> armmmm.updateClawState(BulkReadOnly.ClawState.CLOSE)),
//                                new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_DROP)),
//                                new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INIT)),
//                                new SleepAction(0.5),
//                                new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT)),
//                                new InstantAction(() -> armmmm.setPivotTarget(-1600))
//                        )
//                ));
//            }
//
//            if (gamepad1.y) {
//                ftc.add(new SequentialAction(
//                        new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INIT)),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT)),
//                        new InstantAction(() -> armmmm.setPivotTarget(-1600)),
//                        new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.SAMPLE_DROP)),
//                        new SleepAction(1.5),
//                        new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.BASKET_DROP)),
//                        new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_DROP)),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> armmmm.updateClawState(BulkReadOnly.ClawState.OPEN)),
//                        new SleepAction(1),
//                        new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_PICK)),
//                        new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INTAKE)),
//                        new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT)),
//                        new InstantAction(() -> armmmm.updatePivotState(BulkReadOnly.PivotState.SAMPLE_PRE_PICK))
//                ));
//            }
//
//            if (gamepad1.x) {
//                ftc.add(new ParallelAction(
//                        new ArmPIDUpdater(armmmm),
//                        new SequentialAction(
//                                new InstantAction(() -> armmmm.updateClawState(BulkReadOnly.ClawState.OPEN)),
//                                new InstantAction(() -> armmmm.updateWristState(BulkReadOnly.WristState.SAMPLE_PICK)),
//                                new InstantAction(() -> armmmm.updateElbowState(BulkReadOnly.ElbowState.INIT)),
//                                new SleepAction(0.5),
//                                new InstantAction(() -> armmmm.updateExtensionState(BulkReadOnly.ExtensionState.INIT)),
//                                new InstantAction(() -> armmmm.updatePivotState(BulkReadOnly.PivotState.SAMPLE_PRE_PICK))
//                        )
//                ));
//            }
//
//            // Wrist toggle
//            if (gamepad1.dpad_down) {
//                if (flipped) {
//                    ftc.add(new SequentialAction(
//                            new InstantAction(() -> robot.Wrist.setPosition(globals.Wrist0)),
//                            new InstantAction(() -> robot.Claw.setPosition(globals.ClawOpen)),
//                            new SleepAction(0.1),
//                            new InstantAction(() -> flipped = false)
//                    ));
//                } else {
//                    ftc.add(new SequentialAction(
//                            new InstantAction(() -> robot.Wrist.setPosition(globals.Wrist90)),
//                            new InstantAction(() -> robot.Claw.setPosition(globals.ClawOpen)),
//                            new SleepAction(0.1),
//                            new InstantAction(() -> flipped = true)
//                    ));
//                }
//            }
//
//            // === Telemetry ===
//            telemetry.addLine("======= MODE CONFIG =======");
//            telemetry.addData("Mode", currentMode);
//            telemetry.addData("Bulk Clear Enabled", useBulk);
//            telemetry.addData("Show Loop Time", showLoop);
//
//            telemetry.addLine("======= ARM PID =======");
//            telemetry.addData("Pivot Target", ArmPID.pivotTarget);
//            telemetry.addData("Pivot Pos", robot.pivot_motor.getCurrentPosition());
//            telemetry.addData("Pivot Error", ArmPID.pivotTarget - robot.pivot_motor.getCurrentPosition());
//            telemetry.addData("Ext Target", ArmPID.extensionTarget);
//            telemetry.addData("Ext Pos", robot.extension_motor.getCurrentPosition());
//            telemetry.addData("Ext Error", ArmPID.extensionTarget - robot.extension_motor.getCurrentPosition());
//
//            telemetry.addLine("======= POWER & CURRENT =======");
//            telemetry.addData("Pivot Power", robot.pivot_motor.getPower());
//            telemetry.addData("Ext Power", robot.extension_motor.getPower());
//            telemetry.addData("Pivot Current (A)", robot.pivot_motor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Ext Current (A)", robot.extension_motor.getCurrent(CurrentUnit.AMPS));
//
//            if (showLoop) {
//                long endTime = System.nanoTime();
//                double loopTimeMs = (endTime - startTime) / 1e6;
//
//                double expectedTime = getTargetTimeMs(currentMode);
//
//                telemetry.addData("Expected Loop Time (ms)", expectedTime);
//                telemetry.addData("Actual Loop Time (ms)", String.format("%.2f", loopTimeMs));
//                telemetry.addData("ArmPID Update Time (ms)", String.format("%.2f", arm.getLoopTimeMs()));
//                telemetry.addData("Delta (Actual - Expected)", String.format("%.2f", loopTimeMs - expectedTime));
//            }
//
//
//            telemetry.update();
//
//        }
//    }
//}
