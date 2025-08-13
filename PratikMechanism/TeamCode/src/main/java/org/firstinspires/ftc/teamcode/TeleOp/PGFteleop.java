package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ArmUpdater;
import org.firstinspires.ftc.teamcode.Subsystems.armmm;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Extension Arm (KOOKY BOTZ)", group = "Teleop")
public class PGFteleop extends LinearOpMode {
    public static List<Action> ftc = new ArrayList<>();
    public static BotHardware robot = BotHardware.getInstance();
    private armmm Arm;
    public static boolean Wrist0 = true;
    public static boolean flipped = true;
    public static double strafe = 1, speed = 1, turn = 1;
    public double multiplier = 0;
    private MecanumDrive drive;
    public double botHeading;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        Arm = new armmm(robot, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Gamepad CG1 = new Gamepad();
        Gamepad CG2 = new Gamepad();
        Gamepad PG1 = new Gamepad();
        Gamepad PG2 = new Gamepad();

        while (opModeInInit()) {

            Arm.updatePivotState(armmm.PivotState.INIT);
            Arm.updateExtensionState(armmm.ExtensionState.INIT);
            Arm.updateElbowState(armmm.ElbowState.INIT);
            Arm.updateClawState(armmm.ClawState.INIT);
            Arm.updateWristState(armmm.WristState.INIT);

            Arm.update();

            telemetry.addData("Pivot", robot.pivot_motor.getCurrentPosition());
            telemetry.addData("Extension", robot.extension_motor.getCurrentPosition());
            telemetry.addData("Claw", robot.Claw.getPosition());
            telemetry.addData("Elbow", robot.Elbow.getPosition());
            telemetry.addData("Wrist", robot.Wrist.getPosition());
            telemetry.update();
        }

        waitForStart();

        telemetry.addLine("Started Loop");
        telemetry.update();


        while (opModeIsActive()) {
            PG1.copy(CG1);
            PG2.copy(CG2);
            CG1.copy(gamepad1);
            CG2.copy(gamepad2);

            ftc = updateAction();
            ftc.add(new ArmUpdater(Arm));


/* ------------------------------------------------------ Robot Oriented Control -------------------------------------------------------------------------*/

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x), gamepad1.right_stick_x / 2)
            );
            if (gamepad1.left_trigger > 0.3) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2), gamepad1.right_stick_x / 3)
                );
            }

/*-------------------------------------------------------- Field Oriented Control ----------------------------------------------------------------------*/

//            botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            DriveFieldCentric(-gamepad1.left_stick_x * strafe, -gamepad1.left_stick_y * speed, gamepad1.right_stick_x * turn, botHeading);
//
//            if (gamepad1.back) {
//                robot.imu.resetYaw();
//            }
//
//
//            if (gamepad1.left_trigger > 0) {
//                multiplier = 0.5;
//                DriveFieldCentric(-gamepad1.left_stick_x * multiplier * strafe, -gamepad1.left_stick_y * multiplier * speed, gamepad1.right_stick_x * multiplier * turn, botHeading);
//            } else {
//                DriveFieldCentric(-gamepad1.left_stick_x * strafe, -gamepad1.left_stick_y * speed, gamepad1.right_stick_x * turn, botHeading);
//            }

/* ------------------------------------------------------- Arm Presets =-----------------------------------------------------------------------*/

            Arm.update();

            if (gamepad1.right_bumper) {  /* SAMPLE PRE PICK Position*/
                ftc.add(
                        new ParallelAction(
                                new ArmUpdater(Arm),
                                new SequentialAction(
                                        new InstantAction(() -> Arm.updatePivotState(armmm.PivotState.SAMPLE_PRE_PICK)),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> Arm.updateExtensionState(armmm.ExtensionState.SAMPLE_PICK)),
                                        new SleepAction(1),
                                        new InstantAction(() -> Arm.updateElbowState(armmm.ElbowState.INTAKE)),
                                        new InstantAction(() -> Arm.updateWristState(armmm.WristState.SAMPLE_PICK)),
                                        new InstantAction(() -> Arm.updateClawState(armmm.ClawState.OPEN)),
                                        new SleepAction(0.1)
                                )
                        )
                );
            }


            if (gamepad1.b) { /* SAMPLE PICK Position */
                ftc.add(
                        new ParallelAction(
                                new ArmUpdater(Arm),
                                new SequentialAction(
                                        new InstantAction(() -> Arm.updatePivotState(armmm.PivotState.SAMPLE_PICK)),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> Arm.updateClawState(armmm.ClawState.CLOSE)),
                                        new SleepAction(1),
                                        new InstantAction(() -> Arm.updatePivotState(armmm.PivotState.SAMPLE_PRE_PICK)),
                                        new InstantAction(() -> Arm.updateWristState(armmm.WristState.SAMPLE_DROP)),
                                        new InstantAction(() -> Arm.updateElbowState(armmm.ElbowState.INIT)),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> Arm.updateExtensionState(armmm.ExtensionState.INIT))
                                )
                        )
                );
            }



// Todo ================================================ Wrist Rotation ======================================================================



            if ((gamepad1.dpad_down)) {
                if (flipped) {
                    ftc.add(
                            new SequentialAction(
                                    new InstantAction(() -> robot.Wrist.setPosition(globals.Wrist0)),
                                    new InstantAction(() -> robot.Claw.setPosition(globals.ClawOpen)),
                                    new SleepAction(0.1),
                                    new InstantAction(() -> flipped = false)
                            )
                    );
                }

                else
                {
                    ftc.add(
                            new SequentialAction(
                                    new InstantAction(() -> robot.Wrist.setPosition(globals.Wrist90)),
                                    new InstantAction(() -> robot.Claw.setPosition(globals.ClawOpen)),
                                    new SleepAction(0.1),
                                    new InstantAction(() -> flipped = true)
                            )
                    );
                }
            }


            if (gamepad1.a) { /* heading towards dropping */
                ftc.add(
                        new ParallelAction(
                                new ArmUpdater(Arm),
                                new SequentialAction(
                                        new InstantAction(() -> Arm.updateClawState(armmm.ClawState.CLOSE)),
                                        new InstantAction(() -> Arm.updateWristState(armmm.WristState.SAMPLE_DROP)),
                                        new InstantAction(() -> Arm.updateElbowState(armmm.ElbowState.INIT)),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> Arm.updateExtensionState(armmm.ExtensionState.INIT)),
                                        new InstantAction(() -> Arm.updatePivotState(armmm.PivotState.SAMPLE_DROP))
                                )
                        )
                );
            }


            if (gamepad1.y) {   /* SAMPLE DROP Position */
                ftc.add(new SequentialAction(
                        new InstantAction( () -> Arm.updateExtensionState(armmm.ExtensionState.SAMPLE_DROP)),
                        new SleepAction(1.5),
                        new InstantAction( () -> Arm.updateElbowState(armmm.ElbowState.BASKET_DROP)),
                        new InstantAction( () -> Arm.updateWristState(armmm.WristState.SAMPLE_DROP)),
                        new SleepAction(0.5),
                        new InstantAction( () -> Arm.updateClawState(armmm.ClawState.OPEN)),
                        new SleepAction(2),
                        new InstantAction(() -> Arm.updateWristState(armmm.WristState.SAMPLE_PICK)),
                        new InstantAction(() -> Arm.updateElbowState(armmm.ElbowState.INTAKE)),
                        new InstantAction(() -> Arm.updateExtensionState(armmm.ExtensionState.INIT)) ,
                        new InstantAction( () -> Arm.updatePivotState(armmm.PivotState.SAMPLE_PRE_PICK))

                        )
                );
            }

            if (gamepad1.x) { /* HOME Position */
                ftc.add(
                        new ParallelAction(
                                new ArmUpdater(Arm),
                                new SequentialAction(
                                        new InstantAction(() -> Arm.updateClawState(armmm.ClawState.OPEN)),
                                        new InstantAction(() -> Arm.updateWristState(armmm.WristState.SAMPLE_PICK)),
                                        new InstantAction(() -> Arm.updateElbowState(armmm.ElbowState.INIT)),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> Arm.updateExtensionState(armmm.ExtensionState.INIT)),
                                        new InstantAction(() -> Arm.updatePivotState(armmm.PivotState.SAMPLE_PRE_PICK))
                                )
                        )
                );
            }

/* ------------------------------------------------------- Update PID + FF Control ------------------------------------------------------*/

            Arm.update();

            telemetry.addLine("------ PIVOT ------");
            telemetry.addData("Target", armmm.pivotTarget);
            telemetry.addData("Position", robot.pivot_motor.getCurrentPosition());
            telemetry.addData("Current (A)", robot.pivot_motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Pivot FF", Arm.debugPivotFF);


            telemetry.addLine("------ EXTENSION ------");
            telemetry.addData("Target", armmm.extensionTarget);
            telemetry.addData("Position", robot.extension_motor.getCurrentPosition());
            telemetry.addData("Current (A)", robot.extension_motor.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("------------------------");
            telemetry.addData(" Claw POS : ", robot.Claw.getPosition());
            telemetry.addData(" Elbow POS : ", robot.Elbow.getPosition());
            telemetry.addData(" Wrist POS : ", robot.Wrist.getPosition());

            telemetry.update();
        }

    }

    public void DriveFieldCentric(double x, double y, double rx, double botHeading){

        double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
        double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading));

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower((backLeftPower));
        drive.rightFront.setPower((frontRightPower));
        drive.rightBack.setPower((backRightPower));

    }

    @NonNull
    public static List<Action> updateAction () {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : ftc) {
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
    }
}
