package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class CENTERSTAGE_Auto3 extends LinearOpMode {
    private MecanumLibrary mecanum;
    private MotorHardwareMap motors;
    private VisionSystem vision;
    private boolean prevAutoDrive = false;
    private boolean isAutoDriving = false;

    @Override
    public void runOpMode() {
        motors = new MotorHardwareMap(hardwareMap, telemetry);
        mecanum = new MecanumLibrary(hardwareMap, telemetry);
        mecanum.begin();
        motors.begin();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("front left encoder", mecanum.lF.getCurrentPosition());
            telemetry.addData("front right encoder", mecanum.rF.getCurrentPosition());
            telemetry.addData("back left encoder", mecanum.lB.getCurrentPosition());
            telemetry.addData("back right encoder", mecanum.rB.getCurrentPosition());
            telemetry.update();
        }
    }
}
