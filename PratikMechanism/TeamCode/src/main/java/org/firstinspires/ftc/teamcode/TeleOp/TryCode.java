package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TryCode")
@Config

public class TryCode extends LinearOpMode {
    DcMotorEx motor1 ;
    int motorPose = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "extension_motor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(opModeInInit()){
            motor1.setTargetPosition(0);
            motor1.setPower(0.7);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        waitForStart();

        while (opModeIsActive() ){
            if(gamepad1.a){
                motorPose += 10;
            }

            else if(gamepad1.b){
                motorPose -= 10 ;
            }

            motor1.setTargetPosition(motorPose);
            motor1.setPower(1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("MotorCurrent",motor1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("MotorPse",motor1.getCurrentPosition());
            telemetry.update();
        }

    }
}
