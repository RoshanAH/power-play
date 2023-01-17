package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.rad
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlinx.coroutines.*
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.abs

@TeleOp(group = "testing")
class ConeStack : BaseOpmode() {

  val robot = Jaws()
  var grabbing = false

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.camera.startCamera()
    scope.launch{ 
      robot.camera.apply{
        while(!cameraRunning && !isStopRequested()){
          telemetry.addLine("Waiting for camera to start")
          telemetry.update()
          yield()
        } 
        telemetry.update()
        webcam.setPipeline(robot.camera.alignmentPipeline)
        alignment = if (robot.alliance == "blue") Webcam.Alignment.BLUE_CONES else Webcam.Alignment.RED_CONES
      }
    }

    val dash = FtcDashboard.getInstance()
    telemetry = MultipleTelemetry(telemetry, dash.getTelemetry())
    dash.startCameraStream(robot.camera.webcam, 30.0)
  }

  override fun onStart(scope: CoroutineScope){
    gamepadListener1.a.onPress = {
      scope.launch{
        grabbing = true
        robot.coneStack(4, robot.imu.heading)
        grabbing = false
      }
    }

    gamepadListener1.y.onPress = {
      robot.slides.apply { 
        if (clawPos == open) scope.launch { closeAndRaise() } else open() 
      }
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    if(!robot.camera.cameraRunning) return

    telemetry.addData("distance to tape", robot.camera.closest?.xy?.y)
    telemetry.addData("alliance", robot.alliance)
    telemetry.update()

    robot.slides.apply {
      if (gamepad1.dpad_up) targetPosition = robot.high
      else if (gamepad1.dpad_down) targetPosition = 0.01
      else if (gamepad1.dpad_left) targetPosition = robot.low
      else if (gamepad1.dpad_right) targetPosition = robot.medium
    }

    if(!grabbing){
      robot.drivetrain.drive(
        gamepad1.left_stick_x.toDouble(),
        -gamepad1.left_stick_y.toDouble(),
        gamepad1.right_stick_x * 0.7,
      )
    }
  }
}
