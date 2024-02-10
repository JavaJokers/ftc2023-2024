package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(group = "CENTERSTAGE")
public class CENTERSTAGE_TelOp3 extends LinearOpMode{
    private MecanumLibrary mecanum;
    private MotorHardwareMap motors;
    private VisionSystem vision;
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
        vision = new VisionSystem(telemetry,hardwareMap);
        mecanum.begin();
        motors.begin();
        vision.begin();
        waitForStart();
        while (opModeIsActive()) {
            vision.update();
            boolean d = gamepad1.y;
            if(d&&!prevAutoDrive){isAutoDriving=!isAutoDriving;}
            prevAutoDrive=d;
            if(isAutoDriving){
                telemetry.addLine("is autodriving if triggered");
                if (vision.pixelNum>0 || vision.target.metadata!=null) {
                    if (vision.pixelRecog.getConfidence()>0.8){
                        mecanum.update(0, -0.25, 0, false, false, false);
                        telemetry.addData("greater than .8 accuracy", vision.pixelRecog.getConfidence());
                    }else {
                    mecanum.update(0,0,0,false,false,false);
                    }

                    if (vision.target.metadata!=null&&vision.target.ftcPose.range < 10) {
                        mecanum.update(0, 0, 0, false, false, false);
                        telemetry.addData("range less than 10", vision.target.ftcPose.range);
                    } else if (vision.target.metadata!=null&&vision.target.ftcPose.yaw < 10 && vision.target.ftcPose.yaw > -10) {
                        mecanum.update(0, -0.25, 0, false, false, true);
                        telemetry.addData("yaw if triggered", vision.target.ftcPose.yaw);
                    } else if (vision.target.metadata!=null) {
                        mecanum.update(0, 0, vision.target.ftcPose.yaw / -180, false, false, true);
                        telemetry.addData("yaw if triggered", vision.target.ftcPose.yaw);
                    } else {
                        telemetry.addData("yaw if did not trigger", vision.target.ftcPose.yaw);
                        mecanum.update(0, 0, 0, false, false, false);
                    }
                }
                else {mecanum.update(0,0,0,false,false,false);}
            }
            else{
                targetOne = gamepad2.dpad_up;
                if(targetOne&&!prevTargetOne){
                    elbowTarget = 1;
                }
                prevTargetOne=targetOne;
                targetTwo = gamepad2.dpad_right;
                if(targetTwo&&!prevTargetTwo){
                    elbowTarget = 2;
                }
                prevTargetTwo=targetTwo;
                targetThree = gamepad2.dpad_down;
                if(targetThree&&!prevTargetThree){
                    elbowTarget = 3;
                }
                prevTargetThree=targetThree;

                targetFour = gamepad2.dpad_left;
                if(targetFour&&!prevTargetFour){
                    if(!readyToHang) {
                        elbowTarget = 4;
                        readyToHang =true;
                    } else {
                        elbowTarget = 5;
                        readyToHang = false;
                    }
                }
                prevTargetFour=targetFour;
                float armPower = 0;
                if (gamepad2.left_trigger > 0)
                    armPower = gamepad2.left_trigger;
                else if (gamepad2.right_trigger > 0) {
                    armPower = gamepad2.right_trigger * -1;
                }


                telemetry.addLine("is autodriving if did not trigger");
                mecanum.update(gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, gamepad1.right_stick_x, false,false,gamepad1.left_bumper&&gamepad1.right_bumper);
                motors.update(armPower, gamepad2.x, gamepad2.right_stick_y, gamepad2.left_stick_y, gamepad2.a, (byte)((gamepad2.dpad_down?1:0)+(gamepad2.dpad_left?2:0)+(gamepad2.dpad_up?4:0)+(gamepad2.dpad_right?8:0)),gamepad2.left_bumper&&gamepad2.right_bumper, gamepad2.y, gamepad2.b,elbowTarget);
            }
            /*telemetry.addData("front left encoder", mecanum.lF.getCurrentPosition());
            telemetry.addData("front right encoder", mecanum.rF.getCurrentPosition());
            telemetry.addData("back left encoder", mecanum.lB.getCurrentPosition());
            telemetry.addData("back right encoder", mecanum.rB.getCurrentPosition());
            telemetry.update();*/
        }
        Thread.sleep(1500);
    }
}