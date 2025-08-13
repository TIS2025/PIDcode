package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Control Hub Testing", group = "Test")
public class ControlHubTesting extends LinearOpMode {

    // Hardware references
    private DcMotorEx motor;
    private Servo testServo;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    double motorPower=1;
    @Override
    public void runOpMode() {
        // ========== Init Hardware ==========
        motor = hardwareMap.get(DcMotorEx.class, "motor3");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        testServo = hardwareMap.get(Servo.class, "servo1");

//        colorSensor = hardwareMap.get(ColorSensor.class, "color1");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addLine("Hardware initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== DC Motor Control ==========
            if(gamepad1.a) {
                 motorPower = 1; //-gamepad1.left_stick_y;  // Push forward to move forward
//                motor.setPower(1);
//                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            else if(gamepad1.b) {
                 motorPower = - 1; //-gamepad1.left_stick_y;  // Push forward to move forward
//                motor.setPower(1);
//                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            motor.setPower(motorPower);

            telemetry.addLine("=== DC Motor ===");
//            telemetry.addData("Power", motorPower);

            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("Position (ticks)", motor.getCurrentPosition());
            telemetry.addData("Current (A)", motor.getCurrent(CurrentUnit.AMPS));

          //   ========== Servo Control ==========
            if (gamepad1.a) testServo.setPosition(0.0);
            if (gamepad1.b) testServo.setPosition(0.5);
            if (gamepad1.y) testServo.setPosition(1.0);

//            telemetry.addLine("=== Servo ===");
//            telemetry.addData("Position", testServo.getPosition());
//
//            // ========== Color Sensor ==========
//            telemetry.addLine("=== Color Sensor ===");saaq
//            telemetry.addData("Red", colorSensor.red());
//            telemetry.addData("Green", colorSensor.green());
//            telemetry.addData("Blue", colorSensor.blue());
//            telemetry.addData("Alpha", colorSensor.alpha());
//
//            // ========== Distance Sensor ==========
//            telemetry.addLine("=== Distance Sensor ===");
//            telemetry.addData("Distance (cm)",
//                    String.format("%.2f", distanceSensor.getDistance(DistanceUnit.CM)));

            telemetry.update();
        }
    }
}
//
//package org.firstinspires.ftc.teamcode.Hardware;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.*;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp(name = "Full Hub Test (Ticks+Current)", group = "Test")
//public class ControlHubTesting extends LinearOpMode {
//
//    private DcMotorEx[] motors = new DcMotorEx[8];
//    private Servo[] servos = new Servo[12];
//    private ColorSensor[] colorSensors = new ColorSensor[2];
//    private DistanceSensor[] distanceSensors = new DistanceSensor[2];
//
//    @Override
//    public void runOpMode() {
//
//        // === Initialize Motors ===
//        for (int i = 0; i < motors.length; i++) {
//            String name = "motor" + i;
//            try {
//                motors[i] = hardwareMap.get(DcMotorEx.class, name);
//                motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
//                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            } catch (Exception e) {
//                telemetry.addLine("Missing motor: " + name);
//            }
//        }
//
//        // === Initialize Servos ===
//        for (int i = 0; i < servos.length; i++) {
//            String name = "servo" + i;
//            try {
//                servos[i] = hardwareMap.get(Servo.class, name);
//            } catch (Exception e) {
//                telemetry.addLine("Missing servo: " + name);
//            }
//        }
//
//        // === Initialize Sensors ===
//        for (int i = 0; i < colorSensors.length; i++) {
//            try {
//                colorSensors[i] = hardwareMap.get(ColorSensor.class, "color" + i);
//                distanceSensors[i] = hardwareMap.get(DistanceSensor.class, "distance" + i);
//            } catch (Exception e) {
//                telemetry.addLine("Missing color/distance sensor: " + i);
//            }
//        }
//
//        telemetry.addLine("Hardware Initialized. Press PLAY.");
//        telemetry.update();
//        waitForStart();
//
//        double servoPos = 0.5;
//
//        while (opModeIsActive()) {
//            telemetry.clear();
//
//            // === Motor Test ===
//            double power = 1;//-gamepad1.left_stick_y;
//            for (int i = 0; i < motors.length; i++) {
//                if (motors[i] != null) {
//                    motors[i].setPower(power);
//                    telemetry.addLine("Motor" + i);
//                    telemetry.addData("Power", "%.2f", power);
//                    telemetry.addData("Velocity",motors[i].getVelocity());
//                    telemetry.addData("Encoder Ticks", motors[i].getCurrentPosition());
//                    telemetry.addData("Current (A)", "%.2f", motors[i].getCurrent(CurrentUnit.AMPS));
//                    telemetry.addLine();
//                }
//            }
//
//            // === Servo Test ===
//            if (gamepad1.a) servoPos = 0.0;
//            if (gamepad1.b) servoPos = 0.5;
//            if (gamepad1.y) servoPos = 1.0;
//
//            for (int i = 0; i < servos.length; i++) {
//                if (servos[i] != null) {
//                    servos[i].setPosition(servoPos);
//                    telemetry.addData("Servo" + i, "Position: %.2f", servos[i].getPosition());
//                }
//            }
//
//            // === Color + Distance Sensor Telemetry ===
//            for (int i = 0; i < colorSensors.length; i++) {
//                if (colorSensors[i] != null && distanceSensors[i] != null) {
//                    telemetry.addLine("ColorSensor" + i);
//                    telemetry.addData("R/G/B/A", "%d / %d / %d / %d",
//                            colorSensors[i].red(),
//                            colorSensors[i].green(),
//                            colorSensors[i].blue(),
//                            colorSensors[i].alpha());
//                    telemetry.addData("Distance (cm)", "%.2f",
//                            distanceSensors[i].getDistance(DistanceUnit.CM));
//                }
//            }
//
//            telemetry.update();
//        }
//    }
//}
//
