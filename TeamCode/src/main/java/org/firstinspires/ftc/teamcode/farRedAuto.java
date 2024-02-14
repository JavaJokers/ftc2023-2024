package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class farRedAuto extends LinearOpMode{
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
        double currentTime = time;
        while (opModeIsActive()) {
            if (time-currentTime >= 1 && time-currentTime < 2) {
                mecanum.update(0, -0.5, 0, false, false, false);
            }
            if (time-currentTime >= 2 && time-currentTime <= 3){
                mecanum.update(0, 0, 0.5, false, false, false);
            }
            if(time-currentTime>3&&time-currentTime<=5.4) {
                mecanum.update(0.5, 0, 0, false, false, false);
            }
            if(time-currentTime>5.4&&time-currentTime<=6.4) {
                motors.arm.setTargetPosition(2000);
            }else if (time-currentTime>6.4){
                mecanum.update(0, 0, 0, false, false, false);
                motors.pixelReleaseServo.setPosition(1.0);
            }
            telemetry.addData("time:", time-currentTime);
            telemetry.addData("time-currentTime:" ,(time - currentTime));
            telemetry.update();
        }
    }

}
