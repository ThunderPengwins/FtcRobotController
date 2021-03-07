package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

@TeleOp(name = "Erso", group = "test")
public class Erso extends jeremy{
    //
    public void runOpMode(){
        //
        InitNoOpen();
        //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //
        int startX = xEnc.getCurrentPosition();
        int startYLeft = yEncLeft.getCurrentPosition();
        //int startYRight = yEncRight.getCurrentPosition();
        double startAngle = getRawGyro();
        //
        int lastX = xEnc.getCurrentPosition();
        int lastYLeft = yEncLeft.getCurrentPosition();
        int lastYRight = yEncRight.getCurrentPosition();
        int curX = lastX;
        int curYLeft = lastYLeft;
        //int curYRight = lastYRight;
        //
        //double origin = getAngle();
        double curAngle = getAngle();
        double lastAngle = curAngle;
        //
        double xPos = (FIELD_DIM_DIG / 2) - (ROBOT_WIDTH / 2);
        double yPos = (FIELD_DIM_DIG / -2) + (ROBOT_LENGTH / 2);
        //
        double[] correction;
        //
        packet.put("Initialization", "complete");
        dashboard.sendTelemetryPacket(packet);
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            if (gamepad1.a){
                origin = getRawGyro();
            }
            //
            leftx = gamepad1.left_stick_x;
            lefty = -gamepad1.left_stick_y;
            rightx = gamepad1.right_stick_x;
            //
            fullBaby(leftx, lefty, rightx, 0.5, 1.0);
            //
            lastX = curX;
            lastYLeft = curYLeft;
            //lastYRight = curYRight;
            lastAngle = curAngle;//store old values
            //
            curX = xEnc.getCurrentPosition();
            curYLeft = yEncLeft.getCurrentPosition();
            //curYRight = yEncRight.getCurrentPosition();
            curAngle = getAngle();//retrieve new values
            //
            xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, curAngle);
            yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, curAngle);//run algorithm
            //
            correction = wallTouchCorrect(xPos, yPos, curAngle);//if running against a wall, use to correct position
            xPos = correction[0];
            yPos = correction[1];
            //
            packet = new TelemetryPacket();
            //
            packet.fieldOverlay()
                    /*.setStroke("red")
                    .strokePolygon(rotateRobot(xPos, yPos, curAngle)[1], rotateRobot(xPos, yPos, curAngle)[0])*/
                    .setStroke("black")
                    .fillRect(fUnits(FIELD_DIM_DIG / -2),fUnits(FIELD_DIM_DIG / 6),fUnits(FIELD_DIM_DIG),2);
            rotRect(packet, xPos, yPos, ROBOT_WIDTH, ROBOT_LENGTH, curAngle, "red");
            rotRect(packet, xPos + (Math.sin(Math.toRadians(curAngle)) * ((ROBOT_LENGTH / 2) - 1)), yPos + (Math.cos(Math.toRadians(curAngle)) * ((ROBOT_LENGTH / 2) - 1)), ROBOT_WIDTH, 2, curAngle, "blue");
            //
            packet.put("angle", curAngle);
            packet.put("xPos", xPos);
            packet.put("yPos", yPos);
            packet.put("curX", curX);
            packet.put("curYLeft", curYLeft);
            //packet.put("curYRight", curYRight);
            dashboard.sendTelemetryPacket(packet);
            //
        }
        //
    }
    //
}
