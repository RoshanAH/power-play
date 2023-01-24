package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.acmerobotics.dashboard.config.Config

@TeleOp(group = "testing")
@Config
class ExtensionTest : LinearOpMode(){

 // retract  = 0.7
 // extend   = 0.48
  companion object {
    @JvmField var servoPos = 0.7
  }

  override fun runOpMode(){
    val extension = hardwareMap.servo.get("extension")
    waitForStart()

    while (opModeIsActive()){
      extension.setPosition(servoPos)
    }
  }

 }
