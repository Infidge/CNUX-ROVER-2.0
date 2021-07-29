package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Auto_ROVER", group="Pushbot")
public class Auto_ROVER extends LinearOpMode {

    Hardware_ROVER robot   = new Hardware_ROVER();
    ElapsedTime runtime = new ElapsedTime();

    String side;
    double angle = 0.0;

    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;

    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;

    private int k = 0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        robot.gyro.calibrate();

        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        if (k == 0) {
            robot.lift.setPower(-0.3);

            while (robot.lift.getPower() < 0 && !robot.topLimit.getState()){
                telemetry.addData("Say", "Lifting...");
                telemetry.update();
            }

            robot.lift.setPower(0.0);
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            k++;
        }

        waitForStart();
        if (opModeIsActive()){
            runtime.reset();
            while (opModeIsActive()){
                robot.leftTrack.setPower(1.0);
                robot.rightTrack.setPower(1.0);

                while (runtime.seconds() < 45) {
                    if (robot.front.getDistance(DistanceUnit.CM) < 30){
                                side = "Front";
                                break;
                    }
                    else if (robot.left.getDistance(DistanceUnit.CM) < 30){
                                side = "Left";
                                break;
                    }
                    else if (robot.right.getDistance(DistanceUnit.CM) < 30){
                                 side = "Right";
                                 break;
                    }
                }
                robot.leftTrack.setPower(0.0);
                robot.rightTrack.setPower(0.0);
                if (side == "Front") {
                    angle += 90;
                    rotate(1.0, angle);
                }
                else if (side == "Right") {
                    angle += 90;
                    rotate(1.0, angle);
                }
                else if (side == "Left") {
                    angle -= 90;
                    rotate(1.0, angle);
                }
                drill();
                runtime.reset();
                side = null;
            }
        }

    }

    public void drill (){
        if (opModeIsActive()){
            robot.drill.setPower(1.0);
            robot.lift.setPower(0.3);
            while (robot.lift.getCurrentPosition() < robot.bottomLimit){
                telemetry.addData("Say", "Drilling...");
                telemetry.update();
            }
            robot.lift.setPower(0.0);
            robot.drill.setPower(0.0);
            sleep(150);
            robot.drill.setPower(-1.0);
            robot.lift.setPower(-0.3);
            while (!robot.topLimit.getState()){
                telemetry.addData("Say", "Lifting...");
                telemetry.update();
            }
            robot.lift.setPower(0.0);
            robot.drill.setPower(0.0);
        }
    }

    public void rotate (  double speed, double angle) {

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        robot.leftTrack.setPower(0);
        robot.rightTrack.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        robot.leftTrack.setPower(leftSpeed);
        robot.rightTrack.setPower(rightSpeed);

        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
