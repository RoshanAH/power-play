package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws

@TeleOp
class MainOp : BaseOpmode() {
  val robot = Jaws()
  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope) {
    robot.camera.startCamera()

    scope.launch {
      robot.camera.apply {
        while (!cameraRunning) yield()
        webcam.setPipeline(alignmentPipeline)
      }
    }
  }

  override fun onStart(scope: CoroutineScope) {
    gamepadListener2.apply {
      a.onPress = {
        robot.slides.apply { if (clawPos == open) scope.launch { closeAndRaise() } else open() }
      }

      // dd.onPress = { robot.slides.targetPosition = 0.01 }
      // dl.onPress = { robot.slides.targetPosition = robot.low }
      // dr.onPress = { robot.slides.targetPosition = robot.medium }
      // du.onPress = { robot.slides.targetPosition = robot.high }
    }
  }

  override fun onUpdate(scope: CoroutineScope) {
    val brake = if (gamepad1.right_bumper) 0.3 else (1.0 - robot.slides.position * 0.7)
    robot.drivetrain.drive(
        gamepad1.left_stick_x.toDouble() * brake,
       -gamepad1.left_stick_y.toDouble() * brake,
        gamepad1.right_stick_x.toDouble() * brake * 0.7
    )
    robot.slides.targetPosition -= gamepad2.left_stick_y * 0.025

    robot.slides.apply {
      if (gamepad2.dpad_up) targetPosition = robot.high
      else if (gamepad2.dpad_down) targetPosition = 0.01
      else if (gamepad2.dpad_left) targetPosition = robot.low
      else if (gamepad2.dpad_right) targetPosition = robot.medium
    }
  }
}
