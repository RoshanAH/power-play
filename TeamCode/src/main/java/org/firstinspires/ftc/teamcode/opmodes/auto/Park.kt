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
class Park : BaseOpmode(){
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


    scope.launch{
      while(signal == 0 || !robot.camera.cameraRunning && !isStopRequested()) {
        signal = robot.camera.signalPipeline.signalVal  
        telemetry.addData("signal", signal)
        telemetry.update()
        yield()
      }

      robot.follow{
        val speedUp = to(0.0, 10.0, 0.0)
        val stop = interpolate(speedUp.vf, Pose.zero)
        displace(25.0 - speedUp.displacement.pos.magnitude - stop.displacement.pos.magnitude)
        when(signal) {
          1 -> {
            to(-10.0, 0.0, 0.0)
            displace(28.0)
          }
          3 -> {
            to(10.0, 0.0, 0.0)
            displace(28.0)
          }
        }
        stop()
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
