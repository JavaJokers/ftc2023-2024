
package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(group = "CENTERSTAGE")
public class SampleOpMode extends LinearOpMode{
    //instantiating mecanum and motor libraries/maps
    private sampleMechanum mecanum;
    private sampleMotors motors;
    //declaring variables

    private boolean twenntyFive = false;
    private boolean seventyFive = false;


    @Override
    //where all the code goes
    public void runOpMode() throws InterruptedException {
        //instantiating mecanum and motor libraries/maps
        mecanum = new sampleMechanum(hardwareMap, telemetry);
        motors = new sampleMotors(hardwareMap, telemetry);
        mecanum.begin();
        motors.begin();
        waitForStart();
        //where most of the code goes. this runs on a loop while the opmode is active
        while (opModeIsActive()) {
            //code to set values
            //setting arm power based on triggers
            float armPower = 0;
            if (gamepad2.left_trigger > 0) {
                armPower = gamepad2.left_trigger;
            } else if (gamepad2.right_trigger > 0) {
                armPower = gamepad2.right_trigger * -1;
            }
            //setting 75/25% speeds
            if(gamepad1.left_trigger != 0){
                twenntyFive = true;
            }else{
                twenntyFive = false;
            }
            if(gamepad1.right_trigger != 0){
                seventyFive = true;
            }else{
                seventyFive = false;
            }
            //sending values to mecanum and motors
            mecanum.update(gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, gamepad1.right_stick_x, seventyFive, twenntyFive, gamepad1.left_bumper && gamepad1.right_bumper);
            motors.update(armPower, gamepad2.x);
        }
        Thread.sleep(1500);
    }
}
