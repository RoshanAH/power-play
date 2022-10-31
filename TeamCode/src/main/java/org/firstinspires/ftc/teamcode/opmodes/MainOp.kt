package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws

@TeleOp
class MainOp : BaseOpmode() {
  val robot = Jaws()
  override fun setRobot() = robot

  override fun onStart() {

    gamepadListener1.apply {
      onJoystickMove = {
      }
    }

    gamepadListener2.apply {
      a.onPress = {
        robot.slides.apply { if (clawPos == open) clawPos = close else clawPos = open }
      }

      dd.onPress = { robot.slides.targetPosition = 0.02 }
      dl.onPress = { robot.slides.targetPosition = robot.low }
      dr.onPress = { robot.slides.targetPosition = robot.medium }
      du.onPress = { robot.slides.targetPosition = robot.high }

      onJoystickMove = {
        robot.slides.targetPosition -= gamepad2.left_stick_y * 0.05
      }
    }
  }

  override fun onUpdate(){
        val brake = if (gamepad1.right_bumper) 0.3 else (1.0 - robot.slides.position * 0.7)
        robot.drivetrain.drive(
            gamepad1.left_stick_x.toDouble() * brake,
            -gamepad1.left_stick_y.toDouble() * brake,
            gamepad1.right_stick_x.toDouble() * brake * 0.7
        )
  }
}
