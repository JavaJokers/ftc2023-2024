package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumLibrary;
@TeleOp( group = "CENTERSTAGE")
public class CENTERSTAGE_TelOp1 extends LinearOpMode {
    private MecanumLibrary mecanum;
    @Override
    public void runOpMode(){
        mecanum = new MecanumLibrary(hardwareMap, telemetry);
        mecanum.begin();
        waitForStart();
        while(opModeIsActive()){
            mecanum.update(gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger==1,gamepad1.right_trigger==1,gamepad1.left_bumper&&gamepad1.right_bumper);
        }
    }

}
