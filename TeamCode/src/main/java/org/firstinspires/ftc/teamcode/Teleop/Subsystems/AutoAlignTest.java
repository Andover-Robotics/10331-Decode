package org.firstinspires.ftc.teamcode.Teleop.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Teleop.Bot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="April Tag Auto Align", group="Test")

public class AutoAlignTest extends LinearOpMode {
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, rotation;
    PIDController           pidRotate;
    Bot bot;
    public IMU imu;
    MotorEx fl, bl, br, fr;

    double bearing;


//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Bot.instance = null;
//        bot = Bot.getInstance(this);
//        List<MotorEx> motorList = bot.returnMotors();
//        fl = motorList.get(0);
//        bl = motorList.get(1);
//        br = motorList.get(2);
//        fr = motorList.get(3);
//        imu = bot.imu;
//
//        pidRotate = new PIDController(.003, .0, 0);
//        bot.aprilTag.targetAllianceId=21;
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            TelemetryPacket packet = new TelemetryPacket();
//            bot.aprilTag.findAprilTag();
//            bearing = bot.aprilTag.getBearing();
//            rotate(bearing, 0.3);
//            telemetry.addData("bearing:",bot.aprilTag.getBearing());
//            telemetry.addData("target:",pidRotate.performPID(getAngle()));
//            telemetry.update();
//
//        }
//    }
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;

        bot = Bot.getInstance(this);

        bot.aprilTag.targetAllianceId = 21;
        List<MotorEx> motorList = bot.returnMotors();
        fl = motorList.get(0);
        bl = motorList.get(1);
        br = motorList.get(2);
        fr = motorList.get(3);
        imu = bot.imu;


        while (!isStarted()) {
            TelemetryPacket packet = new TelemetryPacket();
            pidRotate = new PIDController(.003, .0, 0);
            telemetry.addData("PID kp:", 0.003);
            telemetry.update();
        }


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            bot.aprilTag.findAprilTag();
            telemetry.addData("bearing:", bot.aprilTag.getBearing());
            bearing = bot.aprilTag.getBearing();
            rotate(bearing, 0.3);
            telemetry.addData("target:", pidRotate.performPID(getAngle()));
            telemetry.update();


        }
    }


    /** Resets the cumulative angle tracking to zero.*/
    private void resetAngle(){
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.*/
    private double getAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /*** Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right*/
    private void rotate(double degrees, double power){

        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        double rad= angleWrap(Math.toRadians(degrees));
        degrees= Math.toDegrees(rad);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        /*if (degrees < 0) {

            // On right turn we have to get off zero first.
            /*while (opModeIsActive() && getAngle() == 0) {
                bl.set(power);
                fl.set(power);
                br.set(-power);
                fr.set(-power);
                sleep(100);
            }



            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                bl.set(-power);
                fl.set(-power);
                br.set(power);
                fr.set(power);
            }
            while (opModeIsActive() && !pidRotate.onTarget());
        }


        else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                bl.set(-power);
                fl.set(-power);
                br.set(power);
                fr.set(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

         */


        do {
            power = pidRotate.performPID(getAngle()); // power will be + on left turn.
            bl.set(-power);
            fl.set(-power);
            br.set(power);
            fr.set(power);
        } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        bl.set(0);
        fl.set(0);
        br.set(0);
        fr.set(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

}

