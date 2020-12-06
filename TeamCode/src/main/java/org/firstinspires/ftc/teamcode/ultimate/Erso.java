package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Erso", group = "test")
public class Erso extends jeremy{
    //
    public void runOpMode(){
        //
        Init();
        //
        int startX = wobble.getCurrentPosition();
        int startY = YEncoder.getCurrentPosition();
        //
        int lastX = wobble.getCurrentPosition();
        int lastY = YEncoder.getCurrentPosition();
        int curX = lastX;
        int curY = lastY;
        //
        double origin = getAngle();
        double lastAngle = origin;
        //
        double xPos = 0;
        double yPos = 0;
        //
        waitForStartify();
        //
        while(opModeIsActive()){
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
            fullBaby(leftx, lefty, rightx, 0.5, 1.0);
            //
            lastX = curX;
            lastY = curY;
            lastAngle = getAngle();
            curX = wobble.getCurrentPosition();
            curY = YEncoder.getCurrentPosition();
            //
            xPos += getXOdometry(curX - lastX, curY - lastY, lastAngle, origin);
            yPos += getYOdometry(curX - lastX, curY - lastY, lastAngle, origin);
            //
            telemetry.addData("xPos", xPos);
            telemetry.addData("yPos", yPos);
            telemetry.addData("X change", getXOdometry(curX - lastX, curY - lastY, lastAngle, origin));
            telemetry.addData("Y change", getYOdometry(curX - lastX, curY - lastY, lastAngle, origin));
            //telemetry.addData("X Encoder", curX);
            //telemetry.addData("Y Encoder", curY);
            telemetry.addData("Angle", fixAngle(getAngle() - origin));
            telemetry.addData("X Inch Input", (curX - startX) / oconv);
            telemetry.addData("Turn Inches", getTurnInches(oxOffset, fixAngle(getAngle() - origin)));
            telemetry.addData("X offset", ((curX - startX) / oconv) - getTurnInches(oxOffset, getTurnInches(oxOffset, fixAngle(getAngle() - origin))));
            //telemetry.addData("Y offset", ((curY - startY) / oconv) - getTurnInches(oyOffset, getTurnInches(oyOffset, fixAngle(getAngle() - origin))));
            //telemetry.addData("Total displacement", getOdoData(curX - startX, curY - startY, origin)[0]);
            //telemetry.addData("Displacement angle", getOdoData(curX - startX, curY - startY, origin)[1]);
            telemetry.update();
            //
        }
        //
    }
    //
}
