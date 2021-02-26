package org.firstinspires.ftc.teamcode.ultimate;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    double ayaw = 0;
    boolean moveTurning = false;
    //
    double origin = 0;
    //
    public void runOpMode() {
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        Init();
        //
        //backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //
        waitForStartify();
        //
        while (opModeIsActive()){
            //
            if (gamepad1.a){
                origin = getAngle();
                //orchosen = true;
            }
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
                launcher.setPower(.85);
                strafeToPosition(-7.5, .3);
            }
            if(firstdRight && !fdrightpressed){
                turnToAngle(0, .3);
                fdrightpressed = true;
            }else if(!firstdRight && fdrightpressed){
                fdrightpressed = false;
            }
            //
            if(gamepad1.right_trigger > 0){
                resetOrigin();
            }
            //
            runIntake(aButton,yButton,oyButton);
            //
            runIntakeFeeder(firstdLeft, firstdUp);
            //
            runFeed(bButton);
            //
            runLauncherIncr(dUp,dDown,leftBumper, rightBumper);//run launcher with arrows to increment by 5%
            //
            runWobble(-leftTrigger, -rightTrigger);
            //
            keepEmDead(xButton);//run wobble servo
            //
            runTaper(firstdDown);
            //
            telemetry.addData("tape mode", tapeMode);
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
            telemetry.addData("Keeper position", keeper.getPosition());
            telemetry.update();
        }
        //
    }
    //
}
