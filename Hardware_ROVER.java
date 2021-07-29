package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware_ROVER
{

    DcMotor leftTrack;
    DcMotor rightTrack;
    DcMotor lift;
    DcMotor drill;
    
    ModernRoboticsI2cGyro gyro;

    ModernRoboticsI2cRangeSensor front;
    ModernRoboticsI2cRangeSensor right;
    ModernRoboticsI2cRangeSensor left;

    DigitalChannel topLimit;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public int bottomLimit;

    public Hardware_ROVER(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftTrack = hwMap.get(DcMotor.class, "leftTrack");
        leftTrack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftTrack.setPower(0);
        leftTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightTrack = hwMap.get(DcMotor.class, "rightTrack");
        rightTrack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTrack.setPower(0);
        rightTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drill = hwMap.get(DcMotor.class, "drill");
        drill.setDirection(DcMotorSimple.Direction.REVERSE);
        drill.setPower(0);
        drill.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drill.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        front = hwMap.get(ModernRoboticsI2cRangeSensor.class, "frontSensor");
        right = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rightSensor");
        left  = hwMap.get(ModernRoboticsI2cRangeSensor.class, "leftSensor");

        topLimit = hwMap.get(DigitalChannel.class, "topLimit");
        topLimit.setMode(DigitalChannel.Mode.INPUT);
    }
 }