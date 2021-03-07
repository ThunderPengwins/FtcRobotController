package org.firstinspires.ftc.teamcode.ultimateState;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public abstract class Stuart extends LinearOpMode {
    //
    ExpansionHubEx controlHub;
    ExpansionHubEx expansionHub;
    //
    RevBulkData bulkData;
    ExpansionHubMotor frontLeft, frontRight, backLeft, backRight, intake, launcher, wobble;
}
