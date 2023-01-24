package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.acmerobotics.dashboard.config.Config

@TeleOp(group = "testing")
@Config
class ClawTest : LinearOpMode(){

  // open = 0.0
  // close = 0.25
  companion object {
    @JvmField var servoPos = 0.0
  }

  override fun runOpMode(){
    val claw = hardwareMap.servo.get("claw")
    waitForStart()

    while (opModeIsActive()){
      claw.setPosition(servoPos)
    }
  }

 }
