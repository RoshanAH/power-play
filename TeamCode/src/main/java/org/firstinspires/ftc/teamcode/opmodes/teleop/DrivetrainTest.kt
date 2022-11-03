package org.firstinspires.ftc.teamcode.opmodes.teleop

import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.robots.Jaws
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class DrivetrainTest : BaseOpmode() {
  val robot = Jaws()

  override fun setRobot(): Robot = robot

  override suspend fun onStart() {
    gamepadListener1.onJoystickMove = { robot.drivetrain.drive(gamepad1) }
  }
}
