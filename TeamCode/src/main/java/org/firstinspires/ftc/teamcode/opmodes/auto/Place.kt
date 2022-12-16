package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.profiling.buildProfile
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.PSVAController
import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import kotlinx.coroutines.delay
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.sign

@Autonomous
class Place : BaseOpmode(){
  val robot = Jaws()
  override fun setRobot() = robot

  var signal = 0 

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.slides.close()

    robot.camera.startCamera()
    FtcDashboard.getInstance().startCameraStream(robot.camera.webcam, 30.0)

    scope.launch{
      while(!robot.camera.cameraRunning) yield()
      robot.camera.runSignal()
      robot.camera.mount.setPosition(0.56)
    }

  }

  override fun onStart(scope: CoroutineScope){

    robot.slides.close()
    robot.camera.phi = 15.0.deg

    val sideSign = if (robot.side == "left") -1.0 else 1.0
    signal = robot.camera.signalPipeline.signalVal  

    scope.launch{
      while(signal == 0 || !robot.camera.cameraRunning) {
        signal = robot.camera.signalPipeline.signalVal  
        telemetry.addData("signal", signal)
        telemetry.update()
        yield()
      }

      
      launch{
        delay(1000L)
        robot.slides.targetPosition = robot.high
        delay(500L)
        robot.camera.alignment = Webcam.Alignment.ALL
      }

      robot.follow{
        to(0.0, 30.0, 0.0)
        turnTo(-31.0 * sideSign, 0.0, 65.0.deg * sideSign)
      } 

      robot.place() { opModeIsActive() }

      launch{
        delay(1500L)
        robot.slides.targetPosition = 0.01
      }

      delay(1000L)

      robot.follow{
        to(0.0, -20.0, 0.0)
        displace(5.0)
        stop()
      }

      val controller = PSVAController(robot.constants, vi=robot.drivetrain.relativeVel)
      while(opModeIsActive()){
        val dTheta = abs((robot.imu.heading - (-90.0.deg * sideSign)).rad)
        val tw = if (dTheta * 2 * robot.constants.trackRadius >= 0.5)
            sqrt(robot.constants.maxAcceleration * dTheta / robot.constants.trackRadius) *
            sign(((-90.0.deg * sideSign) - robot.imu.heading).rad)
          else 0.0

        controller.update(Pose(Vec2.zero, tw))
        robot.drivetrain.move(robot.constants.psva(controller.wheels, robot.drivetrain.vel))

        if(tw == 0.0) break;
      }
      
      when (signal) {
        2 -> robot.follow{
          to(0.0, 30.0 * sideSign, 0.0)
          stop()
        }
        3 -> robot.follow{
          to(0.0, 30.0 * sideSign, 0.0)
          displace(24.0)
          stop()
        }
      }

      robot.drivetrain.move(DriveValues.zero)

      robot.camera.webcam.stopStreaming()
    }

  }

  override fun onUpdate(scope: CoroutineScope){
    telemetry.addData("heading", robot.imu.heading)
    telemetry.addData("signal", signal)
    telemetry.update()
  }
}
