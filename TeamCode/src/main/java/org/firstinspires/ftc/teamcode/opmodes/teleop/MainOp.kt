package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.deg
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.BaseOpmode

import org.firstinspires.ftc.teamcode.robots.Ptoneigh
import kotlin.math.min

@TeleOp
class MainOp : BaseOpmode() {
  val robot = Ptoneigh({ opModeIsActive() })
  override fun setRobot() = robot
  var cone = 0

  override fun onInit(scope: CoroutineScope) {
    // robot.camera.startCamera()

    // scope.launch {
    //   robot.camera.apply {
    //     while (!cameraRunning) yield()
    //     webcam.setPipeline(alignmentPipeline)
    //   }
    // }
  }

  override fun onStart(scope: CoroutineScope) {
    gamepadListener2.apply {
      a.onPress = {
       robot.turret.apply {
          if (robot.claw.clawPos == robot.claw.open) scope.launch { robot.closeAndRaise() } else robot.claw.open()
        }
      }
      rb.onPress = {
        cone = ((cone + 1) % 5 + 5) % 5
        robot.turret.targetHeight = cone * robot.coneHeight + 0.01
      }
      lb.onPress = {
        cone = ((cone - 1) % 5 + 5) % 5
        robot.turret.targetHeight = cone * robot.coneHeight + 0.01
      }
      dd.onPress = {
        scope.launch{ 
          robot.resetTurret() 
          cone = 0
        }
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope) {
    val manualBrake = if (gamepad1.right_bumper) 0.3 else 1.0
    val slideBrake = 1.0 - robot.turret.height * 0.7

    robot.drivetrain.drive(
        gamepad1.left_stick_x.toDouble() * min(manualBrake, slideBrake),
        -gamepad1.left_stick_y.toDouble() * min(manualBrake, slideBrake),
        gamepad1.right_stick_x.toDouble() * manualBrake * 0.7
    )

    robot.turret.targetHeight -= gamepad2.left_stick_y * 0.01

    robot.turret.apply {
      var pressed = false

      if (gamepad2.dpad_up) {
        targetHeight = robot.high
        robot.claw.targetExtension = 2.0
        pressed = true
      }
      else if (gamepad2.dpad_left) {
        targetHeight = robot.low
        robot.claw.targetExtension = 2.0
        pressed = true
      }
      else if (gamepad2.dpad_right) {
        targetHeight = robot.medium
        robot.claw.targetExtension = 2.0
        pressed = true
      }

      if(pressed){
        cone = 0
      }
    }

    val rightStick = gamepad2.run {
      Vec2(right_stick_x.toDouble(), -right_stick_y.toDouble())
    }
    if(rightStick.magnitude > 0.3){
      robot.turret.targetTheta = (-rightStick.rotate(-90.0.deg)).theta.deg
    }

    // robot.turret.targetTheta += gamepad2.right_stick_x 
    robot.claw.targetExtension += (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() * 0.15


    telemetry.addData("targetTheta", robot.turret.targetTheta)
    telemetry.addData("theta", robot.turret.theta)
    telemetry.update()
  }
}
