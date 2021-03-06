package org.firstinspires.ftc.teamcode.ultimate;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name = "Luke", group = "working")
public class Luke extends jeremy {
    //
    double horiFactor = 1.1;
    //
    boolean aButton;
    boolean bButton;
    boolean yButton;
    boolean xButton;
    boolean oyButton;
    boolean oxButton;
    boolean dUp;
    boolean dDown;
    boolean leftBumper;
    boolean rightBumper;
    boolean firstdLeft;
    boolean firstdRight;
    boolean firstdUp;
    boolean firstdDown;
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
        //backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //
        //angle = getAngle();
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
            bButton = gamepad2.b;//gamepad 2
            yButton = gamepad1.y;
            xButton = gamepad2.x;
            oyButton = gamepad2.y;
            oxButton = gamepad1.x;
            dUp = gamepad2.dpad_up;
            dDown = gamepad2.dpad_down;
            leftBumper = gamepad2.left_bumper;//gamepad 2
            rightBumper = gamepad2.right_bumper;
            firstdRight = gamepad1.dpad_right;//set angle
            firstdLeft = gamepad1.dpad_left;//start and stop intake feeder
            firstdUp = gamepad1.dpad_up;//reverse intake feeder direction
            firstdDown = gamepad1.dpad_down;//move behind line
            leftTrigger = gamepad2.left_trigger;
            rightTrigger = gamepad2.right_trigger;
            //
            if(tapeMode == 0) {
                fullBaby(leftx, lefty, rightx, 0.5, 1.0);//all chassis motion
            }
            //
            if(gamepad1.left_trigger > 0){//strafe for power shot
                //turnToAngle(-8, .3);
                launcher.setPower(psLauncherPower);
                strafeToPosition(-7.5, .3);
                motorsWithEncoders();
            }
            //
            if(oxButton){
                powerFromWall(0.3);//add rpm control here
            }
            //
            /*if(gamepad1.right_stick_button && !fdrightpressed){
                turnToAngle(0, .3);
                fdrightpressed = true;
            }else if(!gamepad1.right_stick_button && fdrightpressed){
                fdrightpressed = false;
            }*/
            //
            if(gamepad1.right_trigger > 0){
                resetOrigin();
            }
            //
            runIntake(aButton,yButton,oyButton);
            //
            //runIntakeFeeder(firstdLeft, firstdUp);
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
            if(firstdDown) {
                runTaper(.3);
            }
            //
            /*if(gamepad1.x){
                autoPower();
            }*/
            //
            //telemetry.addData("Loop speed", System.currentTimeMillis() - time);
            /*telemetry.addData("tape mode", tapeMode);
            telemetry.addData("red value", tape.red());
            telemetry.addData("angle", getAngle());
            telemetry.addData("frontJS", filterfJS(frontJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("leftJS", filterlJS(leftJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("alt leftJS", filterolJS(oleftJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("rightJS", filterrJS(rightJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("alt rightJS", filterorJS(orightJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Feed running", feedRunning);
            telemetry.addData("Stop pending", stopPending);
            telemetry.addData("Stop time", stopTime);
            telemetry.addData("Launcher power", Math.round(1000 * launcherPower) / 10 + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.addData("Intakefeed running", intakeFeederRunning);
            telemetry.addData("Intakefeed power", intakeFeederPower);
            telemetry.addData("Wobble up", wobbleUp.getState());
            telemetry.addData("Keeper position", keeper.getPosition());*/
            //telemetry.addData("Launcher power", Math.round(1000 * launcherPower) / 10 + "%");
            telemetry.addData("rpm Goal", launcherRPM);
            telemetry.addData("rpm toggle", launcherToggle);
            telemetry.addData("rpm avg", rollingAvg);
            telemetry.addData("rpm", rpm);
            //telemetry.addData("angle", getAngle());
            telemetry.update();
            //time = System.currentTimeMillis();
        }
        //
    }
    //
}
