package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class MotorHardwareMap {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DcMotor arm;
    public Servo wrist;
    boolean armlockPosition = false;
    boolean prevArmToggle = false;
    public Servo armlock;
    public MotorHardwareMap (HardwareMap map, Telemetry telem) {
        telemetry = telem;
        hardwareMap = map;
    }
    public void begin () {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armlock = hardwareMap.servo.get("armlock");
        wrist = hardwareMap.servo.get("wrist");
    }
    public void update (float armPower, boolean armToggle, boolean wristLeft, boolean wristRight) {
        arm.setPower(armPower);
        wrist.setPosition(0.0);
        if(armToggle && !prevArmToggle){armlockPosition=!armlockPosition;}
        if (armlockPosition){armlock.setPosition(1.0);}
        else {armlock.setPosition(0.7);}
        //armlock.setPosition(armPower);
        if (wristLeft)
            wrist.setPosition(-90.0);
        else if (wristRight) {
            wrist.setPosition(90.0);
        }
        telemetry.addData("servo: ",armlock.getPosition());
        telemetry.addData("servo" ,wrist.getPosition());
        telemetry.update();
        prevArmToggle = armToggle;
    }
}
