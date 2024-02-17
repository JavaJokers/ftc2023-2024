package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class MotorHardwareMap {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DcMotorEx arm;
    public Servo wrist;
    public Servo launcherServo;
    boolean launcherAcknowledged = false;
    boolean launcherPosition = false;
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
    private boolean servoSpinAcknowledged = false;
    private boolean servoReverseAcknowledged = false;
    private boolean servoDirection = false;
    private boolean intakeSpinning = false;
    private static double armLockLow = 0.76;
    private static double armLockHigh = 1.0;

    private double elbowTargetVelocity = 0;
    private double elbowTargetPosition = 0;


    public MotorHardwareMap (HardwareMap map, Telemetry telem) {
        telemetry = telem;
        hardwareMap = map;
    }
    private void armMovementChange (double armVelocity, int armPosition){
        if(armPosition < arm.getCurrentPosition()){
            armVelocity = Math.abs(armVelocity)*-1;
        }else{
            armVelocity = Math.abs(armVelocity);
        }
        arm.setPower(1.0);
        if(arm.getCurrentPosition()<=(armPosition+50)&&arm.getCurrentPosition()>=(armPosition-50)){
            arm.setTargetPosition(armPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setVelocity(armVelocity);
        }

    }
    private void elbowMovementChange (double elbowVelocity, int elbowPosition){
        if(elbowPosition < elbow.getCurrentPosition()){
            elbowVelocity = Math.abs(elbowVelocity)*-1;
        }else{
            elbowVelocity = Math.abs(elbowVelocity);
        }
        elbowTargetVelocity = elbowVelocity;
        elbowTargetPosition = elbowPosition;
        elbow.setPower(1.0);
        if(elbow.getCurrentPosition()<=(elbowPosition+20)&&elbow.getCurrentPosition()>=(elbowPosition-20)){
            elbow.setTargetPosition(elbowPosition);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setVelocity(elbowVelocity);
        }

    }
    public void begin () {
        //This is where the servos and motors are and commands for them
        launcherServo = hardwareMap.servo.get("launcherServo");
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
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(0.5);
        spin1.setDirection(Servo.Direction.FORWARD);
        spin2.setDirection(Servo.Direction.REVERSE);
    }
    public void update(float armPower, boolean armToggle, float wristMove, float elbowMove, boolean pixelRelease, byte target, boolean armCalibration, boolean servosSpin){
        this.update(armPower, armToggle, wristMove, elbowMove, pixelRelease, target, armCalibration, servosSpin);
    }
    public void update (float armPower, boolean armToggle, float wristMove, float elbowMove, boolean pixelRelease, byte target, boolean armCalibration, boolean servosSpin, boolean servosReverse, int elbowTarget, boolean launcher) {


        if(elbowTarget == 1){ //intake (up)
            elbowTimerActual = elbowTimerStart;
            armMovementChange(150,0);
            elbowMovementChange(50,375);
        }else if(elbowTarget == 2){ // drive (right)
            armMovementChange(200,600);
            elbowMovementChange(100,500);
            elbowTimerActual = elbowTimerStart;
        }else if(elbowTarget == 3){ // place pixel/outtake (down)
            armMovementChange(150,2150);
            elbowMovementChange(100,600);
        }else if(elbowTarget == 4){ // prepare to hang (left)
            elbowTimerActual = elbowTimerStart;
            armMovementChange(150,1800);
            elbowMovementChange(50,0);

        } else if (elbowTarget == 5){ //hang (left again)
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowTimerActual = elbowTimerStart;
            arm.setPower(1.0);
            arm.setTargetPosition(0);
            elbow.setTargetPosition(0);
            armlock.setPosition(armLockLow);
        }

        if(armToggle && !prevArmToggle){armlockPosition=!armlockPosition;}
        if (armlockPosition){
            armlock.setPosition(armLockHigh);
        }
        else {armlock.setPosition(armLockLow);}
        if(servosReverse){
            if(!servoReverseAcknowledged){
                servoDirection=!servoDirection;
                servoReverseAcknowledged = true;
            }
        } else {
            servoReverseAcknowledged = false;
        }

        if(servosSpin){
            if(!servoSpinAcknowledged){
                intakeSpinning=!intakeSpinning;
                servoSpinAcknowledged = true;
            }
        } else {
            servoSpinAcknowledged = false;
        }

        if (intakeSpinning){spin1.setPosition(servoDirection?1.0:0.0);spin2.setPosition(servoDirection?1.0:0.0);}
        else {spin1.setPosition(0.5);spin2.setPosition(0.5);}
        //pixel release
        if(pixelRelease && !prevPixelRelease){pixelReleasePosition=!pixelReleasePosition;}
        if (pixelReleasePosition){pixelReleaseServo.setPosition(1.0);}
        else {pixelReleaseServo.setPosition(0.7);}
        if(launcher) {
            if(!launcherAcknowledged){
                launcherPosition=!launcherPosition;
                launcherAcknowledged = true;
            }
        } else {
            launcherAcknowledged = false;
        }
        
        if (launcherPosition){launcherServo.setPosition(0.75);}
        else {launcherServo.setPosition(launcherServo.getPosition());}
        telemetry.addData("elbowmode", elbow.getMode());
        telemetry.addData("arm currentPosition", arm.getCurrentPosition());
        telemetry.addData("timer",elbowTimerActual);
        telemetry.addData("elbow value", elbowTarget);
        telemetry.addData("pidf",elbow.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("armLock",armlock.getPosition());
        telemetry.addData("target",target);
        //telemetry.addData("arm targetPosition", arm.getTargetPosition());
        telemetry.addData("Elbow Velocity", elbowTargetVelocity);
        telemetry.addData("Elbow Power",elbow.getPower());
        telemetry.addData("elbow targetPosition", elbowTargetPosition);
        telemetry.addData("elbow currentPosition", elbow.getCurrentPosition());
        telemetry.update();
        prevArmToggle = armToggle;
        prevPixelRelease = pixelRelease;
    }
}