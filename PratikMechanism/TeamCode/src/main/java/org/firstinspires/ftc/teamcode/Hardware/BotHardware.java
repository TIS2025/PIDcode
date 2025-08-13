//package org.firstinspires.ftc.teamcode.Hardware;
//
//import androidx.annotation.NonNull;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class BotHardware {
//
//    public IMU imu;
//
//
//    public DcMotorEx pivot_motor, extension_motor;
//    //arm
//    public Servo Elbow,Wrist,Claw;
//
//    public VoltageSensor voltageSensor;
//
//
//    //Todo ========================================================= Robot Setup ===================================================================
//    private static BotHardware instance = null;
//    public boolean enabled;
//
//    public static BotHardware getInstance() {
//        if (instance == null) {
//            instance = new BotHardware();
//        }
//        instance.enabled = true;
//        return instance;
//    }
//
//    public static HardwareMap hardwareMap;
//
//    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP
//        ));
//        imu.initialize(imuParams);
//        BotHardware.hardwareMap = hardwareMap;
//
////        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub"); // or "Expansion Hub"
//
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//
//        Elbow = hardwareMap.get(Servo.class, "Elbow");
//        Wrist = hardwareMap.get(Servo.class, "Wrist");
//        Claw = hardwareMap.get(Servo.class,"Claw");
//
//        pivot_motor = hardwareMap.get(DcMotorEx.class, "pivot_motor");
//        extension_motor = hardwareMap.get(DcMotorEx.class, "extension_motor");
//        extension_motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        pivot_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        extension_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        pivot_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        extension_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//    }
//
//    public void resetencoder() {
//        pivot_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        extension_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        pivot_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        extension_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//    }
//
//    public static double convertToTicks(double value,double radius){
//        return  (2*Math.PI*radius)/8192;
//    }
//
//}

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotHardware {
    public static BotHardware instance;

    public DcMotorEx pivot_motor, extension_motor;
    public Servo Claw, Elbow, Wrist;
    public VoltageSensor voltageSensor;

    // Control and Expansion hubs
    public LynxModule controlHub;
    public LynxModule expansionHub;

    public static BotHardware getInstance() {
        if (instance == null) instance = new BotHardware();
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Motors
        pivot_motor = hardwareMap.get(DcMotorEx.class, "pivot_motor");
        extension_motor = hardwareMap.get(DcMotorEx.class, "extension_motor");
//        extension_motor.setDirection(DcMotorEx.Direction.REVERSE);


        // Servos
        Claw = hardwareMap.get(Servo.class, "Claw");
        Elbow = hardwareMap.get(Servo.class, "Elbow");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        // Voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Get both hubs by name (check configuration names!)
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2"); // Check exact name in FTC Config

        // Set caching mode for performance
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void resetencoder() {
        pivot_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}

