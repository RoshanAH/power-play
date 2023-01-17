package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.PSVAController
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.bisectionMethodSolve
import com.roshanah.jerky.math.rad
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.sin
import com.roshanah.jerky.math.cos
import com.roshanah.jerky.profiling.buildProfile
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import com.acmerobotics.dashboard.FtcDashboard
import kotlinx.coroutines.yield
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.GlobalScope
import kotlin.math.sqrt
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign


@Autonomous
class Place : BaseOpmode(){
  val robot = Jaws().apply { active = { opModeIsActive() } }
  var signal: Int? = null

  // READ THIS FIRST
  // the majority of your editing is just going to be 3 values
  // the tall junction should be good but i just have placeholder values for the other 2 junctions
  // to test a junction you are working on, edit the conePlacements list
  // now i am going to tell you what each parameter of the enum does
  // slidePos is is just the height the slides need to be to place on the junction. you do not need to tune these
  // backwardsDist is how much the robot moves backwards after picking up a cone 
  // placementXVelOffset is an X velocity offset because the robot might veer off the side when moving to the junction
  // angle is just the angle the robot turns to place the on junction

  private enum class Junction(val slidePos: Double, val backwardsDist: Double, val placementXVelOffset: Double, val angle: Angle, val parkDist: Double){
    TALL_X(1.0,                                   /*just these 3 values*/  35.0, -11.0,  110.0.deg, 3.0), // this is the tall junction closest to the cone stack
    MEDIUM(0.7, /*keep these the same as TALL_X but just flip the angle*/  35.0, 11.0, -110.0.deg, 3.0), // this is the medium height junction closest to cone stack
    TALL(1.0, 50.0, 20.0, -130.0.deg, 24.0), // this is the tall junction closest to the robot's starting position
  }

  private val conePlacements = listOf(Junction.TALL_X, Junction.TALL_X, Junction.TALL_X, Junction.MEDIUM, Junction.MEDIUM) // change this around to affect the auto path

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.camera.mount.setPosition(0.53)
    robot.slides.close()
    robot.camera.webcam.setPipeline(robot.camera.signalPipeline)

    scope.launch{
      robot.camera.startCamera()
      while(opModeInInit() && !robot.camera.cameraRunning){
        telemetry.addLine("Waiting for camera to start...")
        telemetry.addData("alliance", robot.alliance)
        telemetry.update()
        yield()
      }

      FtcDashboard.getInstance().startCameraStream(robot.camera.webcam, 30.0)
      
      while(opModeInInit() || signal == null || signal == 0){
        signal = robot.camera.signal
        telemetry.addData("signal", robot.camera.signal)
        telemetry.addData("alliance", robot.alliance)
        telemetry.update()
        yield()
      }
    }
  }

  override fun onStart(scope: CoroutineScope){

    val sideSign = if(robot.side == "right") 1.0 else -1.0

    scope.launch{
      while(opModeIsActive() && signal == null) yield() // wait until the camera has a signal

      robot.camera.webcam.setPipeline(robot.camera.alignmentPipeline) // switch the camera to alignment mode
      robot.camera.alignment = Webcam.Alignment.ALL
      robot.slides.targetPosition = robot.high

      robot.follow{ // approach the junction
        to(-40.0 * sideSign, 5.0, 0.0)
        val angle = 25.0.deg * sideSign
        turnTo(0.0, 35.0, angle)
      }

      robot.place()
      delay(250L)
      // after we place, we want to put the slides down after a second and only look at blue cones in order to pick up from the stack
      // we call launch here because we want to wait a second without blocking the code below it
      launch { 
        robot.camera.alignment = if (robot.alliance == "blue") Webcam.Alignment.BLUE_CONES else Webcam.Alignment.RED_CONES
        delay(500L)
        robot.slides.targetPosition = 0.01
      }

      // back up from the junction, and move and turn to look at the cone stack
      robot.follow(theta=robot.imu.heading){
        to(Pose(-(90.0.deg + theta).dir * 30.0, 0.0))
        turnTo(10.0 * sideSign + cos(theta) * 10.0, 5.0, -theta)
        to(0.0, 30.0, 0.0)
        turnTo(20.0 * sideSign, 0.0, -90.0.deg * sideSign)
      }

      var cycle = 0
      
      while(opModeIsActive() && 30.0 - runtime > 5.0 && cycle <= 4){
        robot.coneStack(4 - cycle, -90.0.deg * sideSign)
        val junction = conePlacements[cycle]
        
        launch{ // after we pick up the cone we need to raise the slides
          delay(500L)
          robot.slides.targetPosition = junction.slidePos
          robot.camera.alignment = Webcam.Alignment.ALL
        }

        robot.follow{ // move to the junction
          val vel = bisectionMethodSolve({
              buildProfile(constants){
                to(0.0, it, 0.0)
                turnTo(junction.placementXVelOffset, it, junction.angle * sideSign)
              }.displacement.y + junction.backwardsDist
            },
            -constants.maxVelocity,
            0.0
          )

          to(0.0, vel, 0.0)
          turnTo(junction.placementXVelOffset, vel, junction.angle * sideSign)
        }

        robot.place()
        delay(250L)
        
        launch{
          delay(500L)
          robot.slides.targetPosition = 0.01
          robot.camera.alignment = Webcam.Alignment.BLUE_CONES
        }

        robot.follow(theta=robot.imu.heading){
          to(Pose((-(90.0.deg + theta).dir) * (10.0 + abs(sin(theta)) * 20.0), 0.0))
        }

        val controller = PSVAController(robot.constants, robot.drivetrain.relativeVel)

        val tTheta = -90.0.deg * sideSign

        while(opModeIsActive()){
          val heading = robot.imu.heading
          val dTheta = tTheta - heading

          robot.constants.apply{
            val tw = sqrt(maxAcceleration / (2 * trackRadius) * abs(dTheta.rad)) * sign(dTheta.rad)
            // val tv = (-90.0.deg - heading).dir * maxVelocity
            controller.update(Pose(Vec2.zero, tw))
            robot.drivetrain.move(psva(wheels(controller.motion), robot.drivetrain.relativeVel))
          }

          if (abs(dTheta.rad) < 0.1) break
        }

        cycle++
      }

      robot.drivetrain.move(DriveValues.zero)
      robot.follow{
        val displacement = (conePlacements[cycle - 1].parkDist + (signal!! - 2) * 18.0) * sideSign
        to(0.0, sqrt(constants.maxAcceleration * abs(displacement)) * sign(displacement), 0.0)
        stop()
      }
      robot.drivetrain.move(DriveValues.zero)
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    telemetry.addData("heading", robot.imu.heading.deg)
    telemetry.update()
  }

  override fun onStop(scope: CoroutineScope){
    robot.camera.webcam.closeCameraDevice() 
  }
}
