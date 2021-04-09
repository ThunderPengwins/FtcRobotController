package org.firstinspires.ftc.teamcode.ultimate;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Rey", group = "working")
public class Rey extends jeremy {
    //
    double horiFactor = 1.1;
    //
    boolean aButton;
    boolean bButton;
    boolean yButton;
    boolean xButton;
    boolean oyButton = false;
    boolean oxButton;
    boolean dUp = false;
    boolean dDown = false;
    boolean dRight;
    boolean leftBumper;
    boolean rightBumper;
    boolean firstdLeft;
    boolean firstdRight;
    boolean firstdUp;
    boolean firstdDown = false;
    float leftTrigger;
    float rightTrigger;
    //
    boolean fdrightpressed;
    //
    public void runOpMode() {
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        InitNoOpen();
        //
        waitForStartify();
        //
        while (opModeIsActive()){
            //
            leftx = gamepad1.left_stick_x;
            lefty = -gamepad1.left_stick_y;
            rightx = gamepad1.right_stick_x;
            //
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            yButton = gamepad1.y;
            xButton = gamepad1.x;
            dRight = gamepad1.dpad_right;
            leftBumper = gamepad1.left_bumper;//gamepad 2
            rightBumper = gamepad1.right_bumper;
            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            //
            if(tapeMode == 0) {
                fullBaby(leftx, lefty, rightx, 0.5, 1.0);//all chassis motion
            }
            //
            if(dRight){
                powerFromWall(0.3);//add rpm control here
            }
            //
            runIntake(aButton,yButton,oyButton);
            //
            runFeed(bButton);
            //
            calcRPM();
            //
            runLauncherSmart(dUp, dDown, leftBumper, rightBumper, rollingAvg);
            //
            runWobble(-leftTrigger, -rightTrigger);
            //
            keepEmDead(xButton);//run wobble servo
            //
            telemetry.addData("rpm Goal", launcherRPM);
            telemetry.addData("rpm toggle", launcherToggle);
            telemetry.addData("rpm avg", rollingAvg);
            telemetry.addData("rpm", rpm);
            //telemetry.addData("angle", getAngle());
            telemetry.update();
        }
        //
    }
    //
}
