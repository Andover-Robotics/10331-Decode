package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public class MainTeleOp extends LinearOpMode {
    Bot bot;
    GamepadEx gp1,gp2;
    private List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        Bot.getInstance(this);
        gp1.readButtons();
        gp2.readButtons();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            //teleop code here





            //updates the readings sent in telemetry packet
            telemetry.update();
            // update runningActions arraylist
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            // send telemetry to dashboard
            dash.sendTelemetryPacket(packet);


        }

    }
}
