package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class CENTERSTAGE_Autonomous1 extends LinearOpMode {
    private MecanumLibrary mecanum;
    private MotorHardwareMap motors;
    private VisionSystem vision;
    private boolean prevAutoDrive = false;
    private boolean isAutoDriving = false;

    @Override
    public void runOpMode() {
        mecanum = new MecanumLibrary(hardwareMap, telemetry);
        motors = new MotorHardwareMap(hardwareMap, telemetry);
        mecanum.begin();
        motors.begin();
        waitForStart();
        mecanum.update(0, 0, 180, false, false, false);
        motors.arm.setTargetPosition(30);
        if (opModeIsActive()) {
            motors.arm.setTargetPosition(30);
            mecanum.update(0, 0, 180, false, false, false);
            while (opModeIsActive()) {
                mecanum.update(0, 50, 0, false, false, false);
            }
        }
    }

}
