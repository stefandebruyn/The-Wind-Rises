package org.firstinspires.ftc.teamcode.vv7797.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.TeamColor;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.OpModeUtil.Plate;
import org.firstinspires.ftc.teamcode.vv7797.opmode.control.AutonomousSuite;

@Autonomous(name = "Auto Blue Front")
public class AutoBlueFront extends LinearOpMode {
    private final TeamColor TEAM_COLOR = TeamColor.BLUE;
    private final Plate PLATE = Plate.FRONT;

    @Override public void runOpMode() {
        AutonomousSuite suite = new AutonomousSuite(this, TEAM_COLOR, PLATE, hardwareMap, telemetry);

        waitForStart();
        suite.autonomous();
    }
}
