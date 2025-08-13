package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name = "Pratik Mechanism", group = "Control")
public class MainCode extends LinearOpMode {
    public static BotHardware robot= BotHardware.getInstance();

    int pivotMin = 0;
    int pivotMax = -1863;

    int extensionMin = 0;
    int extensionMax = 1230;

    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;
    boolean dpadRightPressed = false;
    boolean dpadLeftPressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        robot.resetencoder();

        int pivotTarget = pivotMin;
        int extensionTarget = extensionMin;

        while (opModeIsActive()) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.right_stick_x

                            ),
                                    -gamepad1.left_stick_x

                    )
            );

            drive.updatePoseEstimate();

//------------------------------------------------------------- one task at a time -------------------------------------------------------------

            // D-pad up: Move pivot to MAX
            if (gamepad1.dpad_up && !dpadUpPressed) {
                moveToPosition(robot.pivot_motor, pivotMax, 1);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // D-pad down: Move pivot to MIN
            if (gamepad1.dpad_down && !dpadDownPressed) {
                moveToPosition(robot.pivot_motor, pivotMin, 1);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // D-pad right: Extend to MAX
            if (gamepad1.dpad_right && !dpadRightPressed) {
                moveToPosition(robot.extension_motor, extensionMax, 1);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            // D-pad left: Retract to MIN
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                moveToPosition(robot.extension_motor, extensionMin, 1);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

//------------------------------------------------------------- two task at a time -------------------------------------------------------------

            // --- Y: Extend + Pivot Up ---
            if (gamepad1.y && !yPressed) {
                moveToPosition(robot.extension_motor, extensionMax, 1);
                moveToPosition(robot.pivot_motor, pivotMax, 1);
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            // --- X: Retract + Pivot Down ---
            if (gamepad1.x && !xPressed) {
                moveToPosition(robot.extension_motor, extensionMin, 1);
                moveToPosition(robot.pivot_motor, pivotMin, 1);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // --- B: Extend + Pivot Down ---
            if (gamepad1.b && !bPressed) {
                moveToPosition(robot.extension_motor, extensionMax, 1);
                moveToPosition(robot.pivot_motor, pivotMin, 1);
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            // --- A: Retract + Pivot Up ---
            if (gamepad1.a && !aPressed) {
                moveToPosition(robot.extension_motor, extensionMin, 1);
                moveToPosition(robot.pivot_motor, pivotMax, 1);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }


//            // Clamp target values within min/max range
//            pivotTarget = Math.max(pivotMin, Math.min(pivotMax, pivotTarget));
//            extensionTarget = Math.max(extensionMin, Math.min(extensionMax, extensionTarget));

//            // Apply new targets
//            moveToPosition(pivot_motor, pivotTarget, 0.5);
//            moveToPosition(extension_motor, extensionTarget, 0.6);

            telemetry.addData("Pivot Target", pivotTarget);
            telemetry.addData("Pivot Position", robot.pivot_motor.getCurrentPosition());
            telemetry.addData("Pivot Motor Current : ", robot.pivot_motor.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("-----------------------------------");

            telemetry.addData("Extension Target", extensionTarget);
            telemetry.addData("Extension Position", robot.extension_motor.getCurrentPosition());
            telemetry.addData("Extension Motor Current : ", robot.extension_motor.getCurrent(CurrentUnit.AMPS));

            telemetry.update();

        }
    }

    // Move motor to target position using encoder
    private void moveToPosition(DcMotorEx motor, int target, double power) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        motor.setPower(1);
        motor.setVelocity(3200);

//        if (!motor.isBusy()) {
//            motor.setPower(0);
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
    }
}
