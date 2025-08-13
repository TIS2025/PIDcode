/*-------------------------------------------------------Simple PID Code--------------------*/

//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystems.armPID;
//
//@Config
//@TeleOp(name = "Extension arm PID", group = "Teleop")
//public class PIDcode extends LinearOpMode {
//
//    public static BotHardware robot = BotHardware.getInstance();
//    private armPID arm;
//
//    @Override
//    public void runOpMode() {
//        // Init
//        robot.init(hardwareMap, telemetry);
//        arm = new armPID(robot);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        FtcDashboard.getInstance();
//
//        telemetry.addLine("Custom PID with Dashboard Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        telemetry.addLine("Started Loop");
//        telemetry.update();
//        sleep(1000);
//
//        robot.resetencoder();
//
//        while (opModeIsActive()) {
//
//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.right_stick_x
//
//                            ),
//                            -gamepad1.left_stick_x
//
//                    )
//            );
//
//            drive.updatePoseEstimate();
//
//            // === Gamepad presets for arm ===
//            if (gamepad1.y) {             // Y: Extend + Pivot Up
//                arm.setPivotTarget(-1863);
//                arm.setExtensionTarget(1200);
//            }
//            if (gamepad1.x) {             // X: Retract + Pivot Down
//                arm.setPivotTarget(0);
//                arm.setExtensionTarget(0);
//            }
//            if (gamepad1.b) {             // B: Extend + Pivot Down
//                arm.setPivotTarget(0);
//                arm.setExtensionTarget(1200);
//            }
//            if (gamepad1.a) {             // A: Retract + Pivot Up
//                arm.setPivotTarget(-1863);
//                arm.setExtensionTarget(0);
//            }
//
//            // Run internal PID control
//            arm.update();
//
//            // Telemetry for tuning/debugging
//            telemetry.addLine("--------------- PIVOT ----------------");
//            telemetry.addData("Target", arm.pivotTarget);
//            telemetry.addData("Position", robot.pivot_motor.getCurrentPosition());
//            telemetry.addData("Pivot Motor Current : ", robot.pivot_motor.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.addLine("--------------- EXTENSION --------------");
//            telemetry.addData("Target", arm.extensionTarget);
//            telemetry.addData("Position", robot.extension_motor.getCurrentPosition());
//            telemetry.addData("Extension Motor Current : ", robot.extension_motor.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.addLine("--------------- PID VALUES -------------");
//            telemetry.addData("Pivot kP", arm.pivot_kP);
//            telemetry.addData("Pivot kI", arm.pivot_kI);
//            telemetry.addData("Pivot kD", arm.pivot_kD);
//            telemetry.addData("Ext kP", arm.ext_kP);
//            telemetry.addData("Ext kI", arm.ext_kI);
//            telemetry.addData("Ext kD", arm.ext_kD);
//            telemetry.update();
//        }
//    }
//}

