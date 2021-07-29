package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="TeleOP_ROVER", group="Pushbot")
public class TeleOP_ROVER extends LinearOpMode {

    Hardware_ROVER robot           = new Hardware_ROVER();
    double leftDrive, rightDrive;
    double brake = 1.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        robot.lift.setPower(-0.3);

        while (robot.lift.getPower() < 0 && !robot.topLimit.getState()){
            telemetry.addData("Say", "Lifting...");
            telemetry.update();
        }

        robot.lift.setPower(0.0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            if (Math.abs(gamepad1.right_stick_x) > 0.2){
                leftDrive  = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * 0.5;
                rightDrive = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * 0.5;
            }
            else {
                leftDrive  = -gamepad1.left_stick_y * 0.5;
                rightDrive = -gamepad1.left_stick_y * 0.5;
            }

            if (gamepad1.left_trigger > 0)
                brake = 0.5;
            else if (gamepad1.right_trigger > 0)
                brake = 0.3;
            else brake = 1.0;

            normalize(leftDrive,rightDrive);

            robot.leftTrack.setPower(leftDrive * brake);
            robot.rightTrack.setPower(rightDrive * brake);

            if (gamepad1.left_bumper)
                robot.lift.setPower(0.3);
            else if (gamepad1.right_bumper)
                robot.lift.setPower(-0.3);
            else 
                robot.lift.setPower(0.0);

            if (robot.lift.getPower() > 0 && robot.lift.getCurrentPosition() == robot.bottomLimit)
                robot.lift.setPower(0);
            if (robot.lift.getPower() < 0 && robot.topLimit.getState())
                robot.lift.setPower(0);

            if (gamepad1.dpad_up)
                robot.drill.setPower(1.0);
            if (gamepad1.dpad_down)
                robot.drill.setPower(-1.0);
            if (gamepad1.dpad_left || gamepad1.dpad_right)
                robot.drill.setPower(0.0);

        }
    }

    public void normalize(double left, double right){
        if (opModeIsActive()){
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0){
                left  /= max;
                right /= max;
            }
        }
    }
}
