package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.BitSet;

public class MotorHardwareMap {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DcMotorEx arm;
    public Servo wrist;
    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2,3,2,1);
    boolean armlockPosition = false;
    boolean prevArmToggle = false;
    boolean prevPixelRelease = false;
    boolean pixelReleasePosition = false;
    public Servo armlock;
    public Servo wheelOne;
    public Servo wheelTwo;
    public Servo pixelReleaseServo;
    public DcMotorEx elbow;
    public Servo spin1;
    public Servo spin2;
    private int elbowTimerStart = 50;
    private int elbowTimerActual = elbowTimerStart;
    private boolean firstHang = false;
    private boolean armSet = false;
    private boolean prevArmSet = false;
    private boolean prevToggleBack = false;
    private boolean prevToggleForw = false;
    private boolean prevArmCalibration = false;
    private boolean prevServoSpin = false;
    private boolean prevServoReverse = false;
    private boolean servoDirection = false;
    private boolean intakeSpinning = false;
    private static int armLimitLow = 0;
    private static int armLimitHigh = 2000;
    private static int elbowLimitLow = -3000;
    private static int elbowLimitHigh = 0;
    private static double armLockLow = 0.76;
    private static double armLockHigh = 1.0;
    private static int elbowPosition1 = -1000;
    private static int elbowPosition2 = -1300;
    int mode0Arm = 0;
    int mode1Arm = 0;
    int mode2Arm = 0;
    int mode0Elbow = 0;
    int mode1Elbow = 0;
    int mode2Elbow = 0;
    float mode0Wrist = 0;
    float mode1Wrist = 0;
    float mode2Wrist = 0;

    boolean readyToHang = false;
    public MotorHardwareMap (HardwareMap map, Telemetry telem) {
        telemetry = telem;
        hardwareMap = map;
    }
    public void begin () {
        //This is where the servos and motors are and commands for them
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armlock = hardwareMap.servo.get("armlock");
        wrist = hardwareMap.servo.get("wrist");
        wheelOne = hardwareMap.servo.get("wheelOne");
        wheelTwo = hardwareMap.servo.get("wheelTwo");
        pixelReleaseServo = hardwareMap.servo.get("pixelReleaseServo");
        elbow = (DcMotorEx) hardwareMap.dcMotor.get("elbow");
        spin1 = hardwareMap.servo.get("spin1");
        spin2 = hardwareMap.servo.get("spin2");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elbow.setPositionPIDFCoefficients(2);
        //elbow.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elbow.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        elbow.setPower(0.5);
        spin1.setDirection(Servo.Direction.FORWARD);
        spin2.setDirection(Servo.Direction.REVERSE);
    }
    public void update(float armPower, boolean armToggle, float wristMove, float elbowMove, boolean pixelRelease, byte target, boolean armCalibration, boolean servosSpin){
        this.update(armPower, armToggle, wristMove, elbowMove, pixelRelease, target, armCalibration, servosSpin);
    }
    public void update (float armPower, boolean armToggle, float wristMove, float elbowMove, boolean pixelRelease, byte target, boolean armCalibration, boolean servosSpin, boolean servosReverse, int elbowTarget) {


        if(elbowTarget == 1){
            arm.setPower(0.4);
            elbowTimerActual = elbowTimerStart;
            arm.setTargetPosition(0);
            elbow.setTargetPosition(185);
        }else if(elbowTarget == 2){
            arm.setPower(0.4);
            elbowTimerActual = elbowTimerStart;
            arm.setTargetPosition(500);
            elbow.setTargetPosition(185);
        }else if(elbowTarget == 3){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setVelocity(100);
            arm.setPower(0.4);
            arm.setTargetPosition(1800);

            if(elbowTimerActual == 0){
                elbow.setTargetPosition(0);
            }else{
                elbowTimerActual--;
            }
        }else if(elbowTarget == 4){
                arm.setPower(0.4);
                elbowTimerActual = elbowTimerStart;
                arm.setTargetPosition(1800);
                elbow.setTargetPosition(185);
        } else if (elbowTarget == 5){
            elbowTimerActual = elbowTimerStart;
            arm.setPower(1.0);
            arm.setTargetPosition(0);
            elbow.setTargetPosition(185);
            armlock.setPosition(armLockLow);
        }
        //arm code
        if(armCalibration&&!prevArmCalibration){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPosition(armLimitLow);
            //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elbow.setTargetPosition(elbowLimitLow);
        }
        prevArmCalibration=armCalibration;
        //if(armPower>0&&arm.getTargetPosition()<=armLimitLow){arm.setTargetPosition(armLimitLow);armPower=0;}
        //if(armPower<0&&arm.getTargetPosition()>=armLimitHigh){arm.setTargetPosition(armLimitHigh);armPower=0;}
        if(armPower!=0){arm.setTargetPosition(arm.getCurrentPosition()-(int)(armPower*120));

        }
        //armlock
        if(armToggle && !prevArmToggle){armlockPosition=!armlockPosition;}
        if (armlockPosition){
            armlock.setPosition(armLockHigh);
            //elbow.setTargetPosition(0);
        }
        else {armlock.setPosition(armLockLow);}
        if(servosReverse && !prevServoReverse){servoDirection=!servoDirection;}
        if(servosSpin && !prevServoSpin){intakeSpinning=!intakeSpinning;}
        if (intakeSpinning){spin1.setPosition(servoDirection?1.0:0.0);spin2.setPosition(servoDirection?1.0:0.0);}
        else {spin1.setPosition(0.5);spin2.setPosition(0.5);}
        //pixel release
        if(pixelRelease && !prevPixelRelease){pixelReleasePosition=!pixelReleasePosition;}
        if (pixelReleasePosition){pixelReleaseServo.setPosition(1.0);}
        else {pixelReleaseServo.setPosition(0.7);}
        prevServoSpin=servosSpin;
        telemetry.addData("timer",elbowTimerActual);
        telemetry.addData("elbow value", elbowTarget);
        telemetry.addData("pidf",elbow.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("armLock",armlock.getPosition());
        telemetry.addData("target",target);
        telemetry.addData("arm targetPosition", arm.getTargetPosition());
        telemetry.addData("arm currentPosition", arm.getCurrentPosition());
        telemetry.addData("elbow targetPosition", elbow.getTargetPosition());
        telemetry.addData("elbow currentPosition", elbow.getCurrentPosition());
        telemetry.update();
        prevArmToggle = armToggle;
        prevPixelRelease = pixelRelease;
    }
}
