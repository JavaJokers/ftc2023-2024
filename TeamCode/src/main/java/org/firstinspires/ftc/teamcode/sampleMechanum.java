package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.PIDController;


//@TeleOp(name = "DoNotRun", group = "Libraries")
// @Disabled
public class sampleMechanum {
    public sampleMechanum(HardwareMap map, Telemetry telem) {
        telemetry = telem;
        hardwareMap = map;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
    }
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public static Orientation angles = new Orientation();
    public DcMotor lF;
    public DcMotor lB;
    public DcMotor rF;
    public DcMotor rB;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    public void begin(){
        lF = hardwareMap.dcMotor.get("front_left");
        lB = hardwareMap.dcMotor.get("back_left");
        rF = hardwareMap.dcMotor.get("front_right");
        rB = hardwareMap.dcMotor.get("back_right");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        lF.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(double x, double y, double t, boolean power75, boolean power25, boolean resetOrient) {
        // rotation
        x = -x;
        t = -t;
        angles = imu.getAngularOrientation();
        double x_rotated = x * Math.cos(-angles.firstAngle) - y * Math.sin(-angles.firstAngle);
        double y_rotated = x * Math.sin(-angles.firstAngle) + y * Math.cos(-angles.firstAngle);
        telemetry.addData("angle: ",angles.firstAngle);
        // x, y, theta input mixing
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        frontLeftPower = (y_rotated + x_rotated + t) / denominator;
        backLeftPower = (y_rotated - x_rotated + t) / denominator;
        frontRightPower = (y_rotated - x_rotated - t) / denominator;
        backRightPower = (y_rotated + x_rotated - t) / denominator;
        // Send calculated power to motors
        if (power75) {
            lF.setPower(frontLeftPower * 0.75);
            rF.setPower(frontRightPower * 0.75);
            lB.setPower(backLeftPower * 0.75);
            rB.setPower(backRightPower * 0.75);
        }
        else if (power25) {
            lF.setPower(frontLeftPower * 0.25);
            rF.setPower(frontRightPower * 0.25);
            lB.setPower(backLeftPower * 0.25);
            rB.setPower(backRightPower * 0.25);
        }
        else {
            lF.setPower(frontLeftPower * 0.90);
            rF.setPower(frontRightPower * 0.90);
            lB.setPower(backLeftPower * 0.90);
            rB.setPower(backRightPower * 0.90);
        }
        // reinitialize field oriented
        if (resetOrient) {
            imu.initialize(parameters);
        }
    }
}
