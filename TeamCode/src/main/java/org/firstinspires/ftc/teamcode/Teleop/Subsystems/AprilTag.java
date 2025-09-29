package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AprilTag {
    // default camera offset stuff prob don't need?
    //may need opencv stuff for camera streaming
    //private Position cameraPosition = new Position(DistanceUnit.INCH,
            //0, 0, 0, 0);
    //private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            //0, -90, 0, 0);

    public VisionPortal visionPortal;
    private AprilTagProcessor processor;
// vars to store data from april tag
    public double range ;
    public double bearing;
    public double yaw;
    public int id;


    public AprilTag(OpMode opMode) {
        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(opMode.hardwareMap.get(WebcamName.class, "webcam"), processor);
    }
    public void findAprilTag(){

        List<AprilTagDetection> currentDetections = processor.getDetections();


        for (AprilTagDetection detection : currentDetections){
            if (detection.metadata!=null){
                id = detection.id;
                range = detection.ftcPose.range;
                bearing = detection.ftcPose.bearing;
                yaw = detection.ftcPose.yaw;

            }
        }
    }

    public double getRange(){
        return range;
    }
    public double getBearing() {
        return bearing;
    }
    public double getYaw(){
        return yaw;
    }
    public double getId(){
        return id;
    }
}
