package org.firstinspires.ftc.teamcode.robots


import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.utils.PSVAController
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.deg
import kotlinx.coroutines.delay

import org.firstinspires.ftc.teamcode.components.*
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.readFile
import kotlin.math.sqrt
import kotlin.math.abs
import kotlin.math.sign

class Ptoneigh(val active : () -> Boolean) : Robot(){

    lateinit var drivetrain : Mecanum;
    lateinit var turret : Turret;
    lateinit var claw : Claw;
    // lateinit var camera : Webcam;
    lateinit var imu : IMU;

    private lateinit var voltageSensor: HardwareMap.DeviceMapping<VoltageSensor>

    val low = 0.45
    val medium = 0.7
    val high = 1.0
    val coneHeight = 0.168 / 5

    val alliance: String
    val side: String

    val constants = DriveConstants(
            80.0,
            100.0,
            10.5 * 0.5,
            PSVAConstants(0.0, 0.1, 0.01, 0.0015),
            PSVAConstants(0.0, 0.1, 0.013, 0.002),
            PSVAConstants(0.0, 0.1, 0.017, 0.001),
            DriveValues(1.0, 1.0, 1.0, 1.0)
    )

    init {
        alliance = readFile("alliance.txt")
        side = readFile("side.txt")
    }

    val voltage: Double
        get() = voltageSensor.elementAt(0).getVoltage()

    override fun mapHardware(map: HardwareMap) {
        voltageSensor = map.voltageSensor

        // camera = Webcam(map, "camera", "mount", { turret.height }, { drivetrain.relativeVel.y }).apply { kp = 0.05 }

        // imu = IMU(map, "imu", jawsHubOrientation)

        drivetrain =
                Mecanum(map, "fl", "fr", "bl", "br", { voltage }).apply {
                    trackRadius = constants.trackRadius
                    ticksPerInch = 30.9861111
                }

        turret = Turret(map, "turret", "slides")
        claw = Claw(map, "claw", "extension")
        imu = IMU(map, "imu", tonyHubOrientation)
        addComponents(turret, drivetrain, claw, imu)
    }

    suspend fun resetTurret(){
      if (turret.targetHeight > 0.1) {
        claw.targetExtension = 0.0
        delay(250L)
      }
      turret.targetHeight = 0.0
      turret.targetTheta = 0.0
    }

    suspend fun closeAndRaise(){
        claw.close()
        delay(500L)
        turret.targetHeight += 0.07
    }

    suspend fun forward(inches: Double, tolerance: Double = 0.5, accelScale: Double = 0.5){
      val controller = PSVAController(constants)
      drivetrain.reset()

      while(active()){
        val pos = drivetrain.pos.run { fl + fr + bl + br } * 0.25
        val error = inches - pos
        val tv = sqrt(2 * constants.maxAcceleration * abs(error) * accelScale) * sign(error)

        controller.update(Pose(0.0, tv, 0.0))
        drivetrain.move(constants.sva(controller.motion))

        println(pos)

        if(abs(error) <= tolerance) break
        yield()
      }

      drivetrain.move(DriveValues.zero)
    }

    suspend fun turnTo(heading: Angle, tolerance: Angle = 0.5.deg, accelScale: Double = 0.5){
      val controller = PSVAController(constants)

      while(active()){
        val error = heading - imu.heading
        val tw = sqrt(constants.maxAcceleration / constants.trackRadius * abs(error.rad) * accelScale) * sign(error.rad)

        controller.update(Pose(0.0, 0.0, tw))
        drivetrain.move(constants.sva(controller.motion))

        if(abs(error.rad) <= tolerance.rad) break
        yield()
      }

      drivetrain.move(DriveValues.zero)
    }
}

val tonyHubOrientation = RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)
