package org.firstinspires.ftc.teamcode.Auto.miscRR;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.Auto.miscRR.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d pose;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d startPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pose = startPose;

        pinpoint.recalibrateIMU();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM,
                pose.position.x * 25.4,   // inches → mm if needed
                pose.position.y * 25.4,
                AngleUnit.RADIANS,
                pose.heading.toDouble()
        ));
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        pinpoint.update();   // <-- IMPORTANT

        double x = pinpoint.getPosX(DistanceUnit.MM) / 25.4;
        double y = pinpoint.getPosY(DistanceUnit.MM) / 25.4;
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        pose = new Pose2d(x, y, heading);

        return new PoseVelocity2d(
                new Vector2d(
                        pinpoint.getVelX(DistanceUnit.MM) / 25.4,
                        pinpoint.getVelY(DistanceUnit.MM) / 25.4
                ),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );
    }
}

