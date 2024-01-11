package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.BitSet;

public class MotorHardwareMap {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DcMotor arm;
    public Servo wrist;
    boolean armlockPosition = false;
    boolean prevArmToggle = false;
    boolean prevPixelRelease = false;
    boolean pixelReleasePosition = false;
    public Servo armlock;
    public Servo wheelOne;
    public Servo wheelTwo;
    public Servo pixelReleaseServo;
    public DcMotor elbow;
    public Servo spin1;
    public Servo spin2;
    private boolean armSet = false;
    private boolean prevArmSet = false;
    private boolean prevToggleBack = false;
    private boolean prevToggleForw = false;
    private boolean prevArmCalibration = false;
    private boolean prevServoSpin = false;
    private boolean intakeSpinning = false;
    private static int armLimitLow = 0;
    private static int armLimitHigh = 2000;
    private static int elbowLimitLow = 0;
    private static int elbowLimitHigh = 3000;
    private static double armLockLow = 0.76;
    private static double armLockHigh = 1.0;
    int mode0Arm = 0;
    int mode1Arm = 0;
    int mode2Arm = 0;
    int mode0Elbow = 0;
    int mode1Elbow = 0;
    int mode2Elbow = 0;
    float mode0Wrist = 0;
    float mode1Wrist = 0;
    float mode2Wrist = 0;
    public MotorHardwareMap (HardwareMap map, Telemetry telem) {
        telemetry = telem;
        hardwareMap = map;
    }
    public void begin () {
        //This is where the servos and motors are and commands for them
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armlock = hardwareMap.servo.get("armlock");
        wrist = hardwareMap.servo.get("wrist");
        wheelOne = hardwareMap.servo.get("wheelOne");
        wheelTwo = hardwareMap.servo.get("wheelTwo");
        pixelReleaseServo = hardwareMap.servo.get("pixelReleaseServo");
        elbow = hardwareMap.dcMotor.get("elbow");
        spin1 = hardwareMap.servo.get("spin1");
        spin2 = hardwareMap.servo.get("spin2");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(0.4);
        //wrist.setPosition(wrist.getPosition());
        wrist.scaleRange(0,0.5);
        spin1.setDirection(Servo.Direction.FORWARD);
        spin2.setDirection(Servo.Direction.REVERSE);
    }
    public void update (float armPower, boolean armToggle, float wristMove, float elbowMove, boolean pixelRelease, byte target, boolean armCalibration, boolean servosSpin) {
        if(target!=0){
            armPower=0;
        }
        if((target&1) == 1){
            arm.setTargetPosition(0);
        }
        if(((target&2)>>1) == 1){
            arm.setTargetPosition(600);
        }
        if(((target&4)>>2) == 1){
            arm.setTargetPosition(1850);
        }
        if(((target&8)>>3) == 1){
            arm.setTargetPosition(2000);
        }
        //arm code
        if(armCalibration&&!prevArmCalibration){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setTargetPosition(armLimitLow);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setTargetPosition(elbowLimitLow);
        }
        prevArmCalibration=armCalibration;
        if(armPower>0&&arm.getTargetPosition()<=armLimitLow){arm.setTargetPosition(armLimitLow);armPower=0;}
        if(armPower<0&&arm.getTargetPosition()>=armLimitHigh){arm.setTargetPosition(armLimitHigh);armPower=0;}
        if(armPower!=0){arm.setTargetPosition(arm.getCurrentPosition()-(int)(armPower*120));}
        if(elbowMove<0&&elbow.getTargetPosition()<=elbowLimitLow){elbow.setTargetPosition(elbowLimitLow);elbowMove=0;}
        if(elbowMove>0&&elbow.getTargetPosition()>=elbowLimitHigh){elbow.setTargetPosition(elbowLimitHigh);elbowMove=0;}
        if(elbowMove!=0){elbow.setTargetPosition(elbow.getCurrentPosition()-(int)(elbowMove*40));}
        if(arm.getTargetPosition()<1000){elbow.setTargetPosition(1000);}
        if(arm.getTargetPosition()>1100){elbow.setTargetPosition(1300);}
        //armlock
        if(armToggle && !prevArmToggle){armlockPosition=!armlockPosition;}
        if (armlockPosition){armlock.setPosition(armLockHigh);}
        else {armlock.setPosition(armLockLow);}
        if(servosSpin && !prevServoSpin){intakeSpinning=!intakeSpinning;}
        if (intakeSpinning){spin1.setPosition(1.0);spin2.setPosition(1.0);}
        else {spin1.setPosition(0);spin2.setPosition(0);}
        //pixel release
        if(pixelRelease && !prevPixelRelease){pixelReleasePosition=!pixelReleasePosition;}
        if (pixelReleasePosition){pixelReleaseServo.setPosition(1.0);}
        else {pixelReleaseServo.setPosition(0.7);}
        prevServoSpin=servosSpin;

        //wrist and elbow
        //wrist.setPosition(wrist.getPosition());
        //wrist.setPosition(0);
        wrist.setPosition(0.84);
        telemetry.addData("armLock",armlock.getPosition());
        telemetry.addData("target",target);
        telemetry.addData("elbow", elbowMove);
        telemetry.addData("wrist" ,wristMove);
        telemetry.addData("arm targetPosition", arm.getTargetPosition());
        telemetry.addData("arm currentPosition", arm.getCurrentPosition());
        telemetry.addData("elbow targetPosition", elbow.getTargetPosition());
        telemetry.addData("elbow currentPosition", elbow.getCurrentPosition());
        telemetry.update();
        prevArmToggle = armToggle;
        prevPixelRelease = pixelRelease;
    }
}
