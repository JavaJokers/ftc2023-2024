package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp( group = "CENTERSTAGE")
public class CENTERSTAGE_TelOp2 extends LinearOpMode {
    private MecanumLibrary mecanum;
    private MotorHardwareMap motors;

    @Override
    public void runOpMode() {
        mecanum = new MecanumLibrary(hardwareMap, telemetry);
        motors = new MotorHardwareMap(hardwareMap, telemetry);
        mecanum.begin();
        motors.begin();

        waitForStart();
        while (opModeIsActive()) {
            float elbowMove = gamepad2.right_trigger;
            boolean WristPowerLeft = false;
            boolean WristPowerRight = false;
            if (gamepad2.left_trigger > 0)
                WristPowerLeft = true;
            else if (gamepad2.right_trigger > 0) {
                WristPowerRight = true;
                mecanum.update(gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger == 1, gamepad1.right_trigger == 1, gamepad1.left_bumper && gamepad1.right_bumper);
                //motors.update(gamepad2.left_stick_y, gamepad2.x,gamepad2.left_stick_y, elbowMove,true, (byte)0, false,true);
            }

        }
    }
}
