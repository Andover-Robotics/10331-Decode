package org.firstinspires.ftc.teamcode.Teleop.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
    public int targetAllianceId;
    private int id;
    private double offsetConstant = 276.390625; // difference of height to apriltag from ground and height to camera from ground squared

    /*

AprilTags with the ID 21, 22, 23 are located on each rectangular face of the OBELISK, which is placed outside
of the FIELD and can be used to identify the MOTIF for the MATCH.

Tag ID 20: blue shooting location

Tag ID 24: red shooting location
     */


    public AprilTag(OpMode opMode) {
        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(opMode.hardwareMap.get(WebcamName.class, "webcam"), processor);

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

//        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
        gainControl.setGain(200);

    }
    public void findAprilTag(){

        List<AprilTagDetection> currentDetections = processor.getDetections();
        for (AprilTagDetection detection : currentDetections){
            if (detection.metadata!=null) {
                id = detection.id;
                if (detection.id == targetAllianceId) {
                    range = detection.ftcPose.range;
                    bearing = detection.ftcPose.bearing;
                    yaw = detection.ftcPose.yaw;
                    break;
                }

                }

        }
    }

    public double calcAccurateDis() {
        double dist = Math.sqrt((range*range)-offsetConstant);
        if(Math.abs(bearing)<=5){
            return dist;
        }
        else {
            return Math.cos(Math.toRadians(Math.abs(bearing)))* dist;
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
