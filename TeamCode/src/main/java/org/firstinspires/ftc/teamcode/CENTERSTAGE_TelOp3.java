package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(group = "CENTERSTAGE")
public class CENTERSTAGE_TelOp3 extends LinearOpMode{
    private MecanumLibrary mecanum;
    private MotorHardwareMap motors;
    private boolean prevAutoDrive = false;
    private boolean isAutoDriving = false;
    private boolean prevTargetOne = false;
    private boolean targetOne = false;
    private boolean prevTargetTwo = false;
    private boolean targetTwo = false;
    private boolean prevTargetThree = false;
    private boolean targetThree = false;
    private boolean prevTargetFour = false;
    private boolean targetFour = false;

    boolean readyToHang = false;
    int elbowTarget = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new MecanumLibrary(hardwareMap, telemetry);
        motors = new MotorHardwareMap(hardwareMap, telemetry);
        mecanum.begin();
        motors.begin();
        waitForStart();
        while (opModeIsActive()) {
            targetOne = gamepad2.dpad_up;
            if (targetOne && !prevTargetOne) {
                elbowTarget = 1;
            }
            prevTargetOne = targetOne;
            targetTwo = gamepad2.dpad_right;
            if (targetTwo && !prevTargetTwo) {
                elbowTarget = 2;
            }
            prevTargetTwo = targetTwo;
            targetThree = gamepad2.dpad_down;
            if (targetThree && !prevTargetThree) {
                elbowTarget = 3;
            }
            prevTargetThree = targetThree;

            targetFour = gamepad2.dpad_left;
            if (targetFour && !prevTargetFour) {
                if (!readyToHang) {
                    elbowTarget = 4;
                    readyToHang = true;
                } else {
                    elbowTarget = 5;
                    readyToHang = false;
                }
            }
            prevTargetFour = targetFour;
            float armPower = 0;
            if (gamepad2.left_trigger > 0) {
                armPower = gamepad2.left_trigger;
            } else if (gamepad2.right_trigger > 0) {
                armPower = gamepad2.right_trigger * -1;
            }


            mecanum.update(gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, gamepad1.right_stick_x, false, false, gamepad1.left_bumper && gamepad1.right_bumper);
            motors.update(armPower, gamepad2.x, gamepad2.right_stick_y, gamepad2.left_stick_y, gamepad2.a, (byte) ((gamepad2.dpad_down ? 1 : 0) + (gamepad2.dpad_left ? 2 : 0) + (gamepad2.dpad_up ? 4 : 0) + (gamepad2.dpad_right ? 8 : 0)), gamepad2.left_bumper && gamepad2.right_bumper, gamepad2.y, gamepad2.b, elbowTarget,gamepad1.a);
        }    /*telemetry.addData("front left encoder", mecanum.lF.getCurrentPosition());
            telemetry.addData("front right encoder", mecanum.rF.getCurrentPosition());
            telemetry.addData("back left encoder", mecanum.lB.getCurrentPosition());
            telemetry.addData("back right encoder", mecanum.rB.getCurrentPosition());
            telemetry.update();*/
        Thread.sleep(1500);
    }
}