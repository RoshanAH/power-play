package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlin.math.min

@TeleOp
class MainOp : BaseOpmode() {
  val robot = Jaws()
  override fun setRobot() = robot
  var placing = false
  var cone = 0

  override fun onInit(scope: CoroutineScope) {
    robot.camera.startCamera()

    scope.launch {
      robot.camera.apply {
        while (!cameraRunning && !isStopRequested()) yield()
        webcam.setPipeline(alignmentPipeline)
        alignment = Webcam.Alignment.ALL
      }
    }
  }

  override fun onStart(scope: CoroutineScope) {
    gamepadListener2.apply {
      a.onPress = {
       robot.slides.apply { 
          if (clawPos == open) scope.launch { closeAndRaise() } else open() 
        }
      }
    }
    gamepadListener1.apply{
      lb.onPress = {
        scope.launch{
          placing = true
          robot.place(condition={ opModeIsActive() && !gamepad1.right_stick_button && !gamepad1.left_stick_button })
          placing = false
        }
      }
    }

    gamepadListener2.apply {
      rb.onPress = {
        cone = ((cone + 1) % 5 + 5) % 5
        robot.slides.targetPosition = cone * robot.coneHeight + 0.01
      }
      lb.onPress = {
        cone = ((cone - 1) % 5 + 5) % 5
        robot.slides.targetPosition = cone * robot.coneHeight + 0.01
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope) {
    val manualBrake = if (gamepad1.right_bumper) 0.3 else 1.0
    val slideBrake = 1.0 - robot.slides.position * 0.7

    if(!placing){
      robot.drivetrain.drive(
          gamepad1.left_stick_x.toDouble() * min(manualBrake, slideBrake),
          -gamepad1.left_stick_y.toDouble() * min(manualBrake, slideBrake),
          gamepad1.right_stick_x.toDouble() * manualBrake * 0.7
      )
      robot.slides.targetPosition -= gamepad2.left_stick_y * 0.025
      robot.slides.apply {
        if (gamepad2.dpad_up) {
          targetPosition = robot.high
          cone = 0
        }
        else if (gamepad2.dpad_down) {
          targetPosition = 0.01
          cone = 0
        }
        else if (gamepad2.dpad_left) {
          targetPosition = robot.low
          cone = 0
        }
        else if (gamepad2.dpad_right) {
          targetPosition = robot.medium
          cone = 0
        }
      }
    }

    telemetry.addData("robot velocity", robot.camera.robotVel())
    telemetry.update()
  }

  override fun onStop(scope: CoroutineScope){
    robot.camera.webcam.closeCameraDevice() 
  }

}
