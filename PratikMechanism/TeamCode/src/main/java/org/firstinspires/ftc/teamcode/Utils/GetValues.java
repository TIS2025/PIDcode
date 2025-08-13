package org.firstinspires.ftc.teamcode.Utils;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.BotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="GetValuesPratik", group="TeleOp")
public class GetValues extends LinearOpMode {
    public double botHeading;
    public static BotHardware robot= BotHardware.getInstance();

    public double multiplier=1;
    public double strafe = 0.7, speed = 0.7, turn = 0.7;

    int pivotTarget = 0;
    int extensionTarget = 0;

    @Override
    public void runOpMode() {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot.init(hardwareMap,telemetry);
        
        robot.pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extension_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.extension_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

            robot.Claw.setPosition(0.5);
            robot.Elbow.setPosition(0.5);
            robot.Wrist.setPosition(0.5);

            robot.pivot_motor.setTargetPosition(0);
            robot.pivot_motor.setPower(1);
            robot.pivot_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.extension_motor.setTargetPosition(0);
            robot.extension_motor.setPower(0);
            robot.extension_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            telemetry.addData("Pivot Encoder", robot.pivot_motor.getCurrentPosition());
            telemetry.addData("Extension Encoder", robot.extension_motor.getCurrentPosition());
            telemetry.addLine("Manually move the arms to full range.");
            telemetry.addLine("Press START when ready.");
            telemetry.update();
        }

        waitForStart();

        pivotTarget = robot.pivot_motor.getCurrentPosition();
        extensionTarget = robot.extension_motor.getCurrentPosition();

        while (opModeIsActive()) {


// TODO =============================================== INIT ===========================================================

            if (gamepad1.a) {
                pivotTarget -= 5;
            } else if (gamepad1.b) {
                pivotTarget += 5;
            }

            if (gamepad1.right_bumper) {
                extensionTarget -= 20;
            } else if (gamepad1.left_bumper) {
                extensionTarget += 20;
            }

            if(gamepad1.x) {
                robot.Elbow.setPosition(robot.Elbow.getPosition() - 0.01);
            } else if(gamepad1.y){
                robot.Elbow.setPosition(robot.Elbow.getPosition() + 0.01);
            }

            if(gamepad1.dpad_up) {
                robot.Wrist.setPosition(robot.Wrist.getPosition() - 0.01);
            } else if(gamepad1.dpad_down){
                robot.Wrist.setPosition(robot.Wrist.getPosition() + 0.01);
            }

            if(gamepad1.dpad_right) {
                robot.Claw.setPosition(robot.Claw.getPosition() - 0.01);
            } else if(gamepad1.dpad_left){
                robot.Claw.setPosition(robot.Claw.getPosition() + 0.01);
            }

            // Move motors to target positions
            moveToPosition(robot.pivot_motor, pivotTarget, 1);
            moveToPosition(robot.extension_motor, extensionTarget, 1);

            drive.updatePoseEstimate();

// Todo =========================================================== Robot Oriented ======================================================================

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y*speed,-1,1),3),
                                    Math.pow(Range.clip(-gamepad1.left_stick_x*strafe,-1,1),3)),
                            Math.pow(Range.clip(-gamepad1.right_stick_x*turn,-1,1),3))
            );

            if (gamepad1.left_stick_x>0.3){
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y*speed*multiplier,-1,1),3),
                                        Math.pow(Range.clip(gamepad1.left_stick_x*strafe*multiplier,-1,1),3)),
                                Math.pow(Range.clip(-gamepad1.right_stick_x*turn*multiplier,-1,1),3))
                );
            }

// Todo ===============================================================TELEMETRY======================================================================


            telemetry.addData("Pivot Motor Current : ", robot.pivot_motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(" Pivot Motor POS : ", robot.pivot_motor.getCurrentPosition());

            telemetry.addLine("--------------------------------------");

            telemetry.addData(" Extension Motor POS : ", robot.extension_motor.getCurrentPosition());
            telemetry.addData(" Extension Motor Current : ", robot.extension_motor.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("--------------------------------------");

            telemetry.addData(" Claw POS : ", robot.Claw.getPosition());
            telemetry.addData(" Elbow POS : ", robot.Elbow.getPosition());
            telemetry.addData(" Wrist POS : ", robot.Wrist.getPosition());

            telemetry.update();

        }

    }

    private void moveToPosition(DcMotorEx motor, int targetPosition, double power) {
//        if (motor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
//            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        }
        motor.setTargetPosition(targetPosition);
        motor.setPower(power);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }


}

