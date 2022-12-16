package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws

@TeleOp
class MainOp : BaseOpmode() {
  val robot = Jaws()
  override fun setRobot() = robot
  var placing = false
  var coneStack = 0

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
        robot.slides.apply { 
          if (clawPos == open) scope.launch { closeAndRaise() } else open() 
        }
      }
      rb.onPress = {
        robot.slides.apply{
          coneStack += 1
          coneStack = (coneStack % 5 + 5) % 5
          targetPosition = 0.01 + coneStack * robot.coneHeight
        }
      }

      lb.onPress = {
        robot.slides.apply{
          coneStack -= 1
          coneStack = (coneStack % 5 + 5) % 5
          targetPosition = 0.01 + coneStack * robot.coneHeight
        }
      }
    }
    // gamepadListener1.apply{
    //   du.onPress = {
    //     scope.launch{
    //       placing = true
    //       robot.alignAndPlace(8.0)
    //       placing = false
    //     }
    //   }
    // }
  }

  override fun onUpdate(scope: CoroutineScope) {
    val brake = if (gamepad1.right_bumper) 0.3 else (1.0 - robot.slides.position * 0.7)
    if(!placing){
      robot.drivetrain.drive(
          gamepad1.left_stick_x.toDouble() * brake,
          -gamepad1.left_stick_y.toDouble() * brake,
          gamepad1.right_stick_x.toDouble() * brake * 0.7
          )
        robot.slides.targetPosition -= gamepad2.left_stick_y * 0.025
        robot.slides.apply {
          if (gamepad2.dpad_up){
            targetPosition = robot.high
            coneStack = 0
          } 
          else if (gamepad2.dpad_down){
            targetPosition = 0.01
            coneStack = 0
          } 
          else if (gamepad2.dpad_left){
            targetPosition = robot.low
            coneStack = 0
          } 
          else if (gamepad2.dpad_right){
            targetPosition = robot.medium
            coneStack = 0
          } 
        }
    }

    telemetry.addData("slide position", robot.slides.position)
    telemetry.update()
  }
}
