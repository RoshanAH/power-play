package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

@TeleOp(group = "testing")

class IntakeTest : LinearOpMode(){

  override fun runOpMode(){
    val left = hardwareMap.dcMotor.get("left")
    val right = hardwareMap.dcMotor.get("right")

    waitForStart()

    while(opModeIsActive()){
      left.setPower(gamepad1.right_stick_y.toDouble())
      right.setPower(-gamepad1.right_stick_y.toDouble())
    }
  }

}
