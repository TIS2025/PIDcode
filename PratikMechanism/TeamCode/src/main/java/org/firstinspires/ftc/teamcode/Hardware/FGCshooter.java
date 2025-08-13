package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@TeleOp(name = "Shooter FGC", group = "Linear Opmode")
public class FGCshooter extends LinearOpMode {

    private DcMotorEx shooterFL, shooterFR, shooterBL, shooterBR;
    private Servo servo;
    private boolean shooterOn = false;

    @Override
    public void runOpMode() {
        // Hardware mapping
        shooterFL = hardwareMap.get(DcMotorEx.class, "FL");
        shooterFR = hardwareMap.get(DcMotorEx.class, "FR");
        shooterBL = hardwareMap.get(DcMotorEx.class, "BL");
        shooterBR = hardwareMap.get(DcMotorEx.class, "BR");
        servo = hardwareMap.get(Servo.class, "servo1");


        // Set motor direction
        shooterFL.setDirection(DcMotor.Direction.REVERSE);
        shooterFR.setDirection(DcMotor.Direction.FORWARD);
        shooterBL.setDirection(DcMotor.Direction.FORWARD);
        shooterBR.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders (for telemetry only)
//        shooterFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter ready. Press A to start, B to stop.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Start shooter
            if (gamepad1.a) {
                shooterOn = true;
            }

            // Stop shooter
            if (gamepad1.b) {
                shooterOn = false;
            }

            if (gamepad1.dpad_right) servo.setPosition(0.0);
            if (gamepad1.dpad_down) servo.setPosition(0.5);
            if (gamepad1.dpad_left) servo.setPosition(1.0);

            double power = shooterOn ? 0.8 : 0.0;

            shooterFL.setPower(power);
            shooterFR.setPower(power);
            shooterBL.setPower(power);
            shooterBR.setPower(power);

            // Telemetry: encoder position, velocity, and current
            telemetry.addData("Shooter ON?", shooterOn);
            telemetry.addLine("FL | Pos: " + shooterFL.getCurrentPosition()
                    + " | Vel: " + shooterFL.getVelocity()
                    + " | Curr: " + shooterFL.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("FR | Pos: " + shooterFR.getCurrentPosition()
                    + " | Vel: " + shooterFR.getVelocity()
                    + " | Curr: " + shooterFR.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("BL | Pos: " + shooterBL.getCurrentPosition()
                    + " | Vel: " + shooterBL.getVelocity()
                    + " | Curr: " + shooterBL.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("BR | Pos: " + shooterBR.getCurrentPosition()
                    + " | Vel: " + shooterBR.getVelocity()
                    + " | Curr: " + shooterBR.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("=== Servo ===");
            telemetry.addData("Position", servo.getPosition());

            telemetry.update();
        }
    }
}



//package org.firstinspires.ftc.teamcode.Hardware;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//
//@Config
//@TeleOp(name = "Shooter FGC RPM", group = "Linear Opmode")
//public class FGCshooter extends LinearOpMode {
//
//    private DcMotorEx shooterFL, shooterFR, shooterBL, shooterBR;
//    private boolean shooterOn = false;
//
//    // Editable in dashboard
//    public static double targetRPM = 3000;  // desired RPM
//    public static final int TICKS_PER_REV = 28;  // adjust if your motor has a gearbox
//
//    @Override
//    public void runOpMode() {
//        // Hardware mapping
//        shooterFL = hardwareMap.get(DcMotorEx.class, "FL");
//        shooterFR = hardwareMap.get(DcMotorEx.class, "FR");
//        shooterBL = hardwareMap.get(DcMotorEx.class, "BL");
//        shooterBR = hardwareMap.get(DcMotorEx.class, "BR");
//
//        // Set motor direction
//        shooterFL.setDirection(DcMotor.Direction.REVERSE);
//        shooterFR.setDirection(DcMotor.Direction.FORWARD);
//        shooterBL.setDirection(DcMotor.Direction.FORWARD);
//        shooterBR.setDirection(DcMotor.Direction.REVERSE);
//
//        // Use encoders for velocity control
//        shooterFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addLine("Shooter ready. Press A to start (velocity mode), B to stop.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Toggle shooter
//            if (gamepad1.a) shooterOn = true;
//            if (gamepad1.b) shooterOn = false;
//
//            double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
//
//            if (shooterOn) {
//                shooterFL.setVelocity(ticksPerSec);
//                shooterFR.setVelocity(ticksPerSec);
//                shooterBL.setVelocity(ticksPerSec);
//                shooterBR.setVelocity(ticksPerSec);
//            } else {
//                shooterFL.setVelocity(0);
//                shooterFR.setVelocity(0);
//                shooterBL.setVelocity(0);
//                shooterBR.setVelocity(0);
//            }
//
//            // Telemetry
//            telemetry.addData("Shooter ON?", shooterOn);
//            telemetry.addLine("Target RPM: " + targetRPM + " | Ticks/sec: " + ticksPerSec);
//
//            telemetry.addLine("FL | Pos: " + shooterFL.getCurrentPosition()
//                    + " | Vel: " + shooterFL.getVelocity()
//                    + " | Curr: " + shooterFL.getCurrent(CurrentUnit.AMPS));
//            telemetry.addLine("FR | Pos: " + shooterFR.getCurrentPosition()
//                    + " | Vel: " + shooterFR.getVelocity()
//                    + " | Curr: " + shooterFR.getCurrent(CurrentUnit.AMPS));
//            telemetry.addLine("BL | Pos: " + shooterBL.getCurrentPosition()
//                    + " | Vel: " + shooterBL.getVelocity()
//                    + " | Curr: " + shooterBL.getCurrent(CurrentUnit.AMPS));
//            telemetry.addLine("BR | Pos: " + shooterBR.getCurrentPosition()
//                    + " | Vel: " + shooterBR.getVelocity()
//                    + " | Curr: " + shooterBR.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.update();
//        }
//    }
//}

