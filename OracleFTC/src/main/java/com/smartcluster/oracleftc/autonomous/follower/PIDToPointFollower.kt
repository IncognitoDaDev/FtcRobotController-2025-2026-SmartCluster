package com.smartcluster.oracleftc.autonomous.follower

import com.smartcluster.oracleftc.autonomous.MecanumDrive
import com.smartcluster.oracleftc.commands.Command
import com.smartcluster.oracleftc.math.Pose2d
import com.smartcluster.oracleftc.math.PoseVelocity2d
import com.smartcluster.oracleftc.math.PoseVelocity2dDual
import com.smartcluster.oracleftc.math.Time
import com.smartcluster.oracleftc.math.Vector2d
import com.smartcluster.oracleftc.math.control.MecanumKinematics
import com.smartcluster.oracleftc.math.control.MecanumKinematics.WheelVelocities
import com.smartcluster.oracleftc.math.control.MotorFeedforward
import com.smartcluster.oracleftc.math.control.PIDController
import java.util.function.Supplier

class PIDToPointFollower(
    drive: MecanumDrive,
    private val feedforward: MotorFeedforward,
    private val kinematics: MecanumKinematics,
    val parameters: PIDToPointFollowerParameters
) :
    Follower(drive), HoldCapableFollower, LineCapableFollower {
    private val axialPIDController = PIDController(
        parameters.axialGain, 0.0, parameters.axialVelGain,
        parameters.axialLowPassGain
    )
    private val lateralPIDController = PIDController(
        parameters.lateralGain, 0.0, parameters.lateralVelGain,
        parameters.lateralLowPassGain
    )
    private val headingPIDController = PIDController(
        parameters.headingGain, 0.0, parameters.headingVelGain,
        parameters.headingLowPassGain
    )

    constructor(
        drive: MecanumDrive,
        feedforward: MotorFeedforward,
        kinematics: MecanumKinematics,
        axialGain: Double,
        axialVelGain: Double,
        axialLowPassGain: Double,
        lateralGain: Double,
        lateralVelGain: Double,
        lateralLowPassGain: Double,
        headingGain: Double,
        headingVelGain: Double,
        headingLowPassGain: Double
    ) : this(
        drive,
        feedforward,
        kinematics,
        PIDToPointFollowerParameters(
            axialGain, axialVelGain, axialLowPassGain, lateralGain,
            lateralVelGain, lateralLowPassGain, headingGain, headingVelGain,
            headingLowPassGain
        )
    )

    override fun hold(pose: Pose2d): Command {
        return Command.builder()
            .init {
                axialPIDController.reset()
                lateralPIDController.reset()
                headingPIDController.reset()
            }
            .update {
                drive.update()
                val currentPose = drive.pose.value()
                val velocityError = -drive.pose.velocity().value()


                val wheelVelocities: WheelVelocities<Time> = kinematics.inverse(
                    PoseVelocity2dDual.constant(
                        PoseVelocity2d(
                            Vector2d(
                                axialPIDController.update(
                                    pose.x,
                                    currentPose.x,
                                    velocityError.linearVel.x
                                ),
                                lateralPIDController.update(
                                    pose.y,
                                    currentPose.y,
                                    velocityError.linearVel.y
                                )
                            ),
                            headingPIDController.update(
                                pose.heading.log(),
                                currentPose.heading.log(),
                                velocityError.angVel
                            )
                        ), 2
                    )
                )
                drive.setMotorPowers(
                    feedforward.update(wheelVelocities.rightFront),
                    feedforward.update(wheelVelocities.rightBack),
                    feedforward.update(wheelVelocities.leftFront),
                    feedforward.update(wheelVelocities.leftBack)
                )
            }
            .build()
    }


    class PIDToPointFollowerParameters(
        val axialGain: Double, val axialVelGain: Double,
        val axialLowPassGain: Double, val lateralGain: Double,
        val lateralVelGain: Double,
        val lateralLowPassGain: Double, val headingGain: Double,
        val headingVelGain: Double,
        val headingLowPassGain: Double
    )

    override fun lineTo(pose: Pose2d): Command {
        return Command.builder()
            .init {
                axialPIDController.reset()
                lateralPIDController.reset()
                headingPIDController.reset()
            }
            .update {
                drive.update()
                val currentPose = drive.pose.value()
                val velocityError = -drive.pose.velocity().value()


                val wheelVelocities: WheelVelocities<Time> = kinematics.inverse(
                    PoseVelocity2dDual.constant(
                        PoseVelocity2d(
                            Vector2d(
                                axialPIDController.update(
                                    pose.x,
                                    currentPose.x,
                                    velocityError.linearVel.x
                                ),
                                lateralPIDController.update(
                                    pose.y,
                                    currentPose.y,
                                    velocityError.linearVel.y
                                )
                            ),
                            headingPIDController.update(
                                pose.heading.log(),
                                currentPose.heading.log(),
                                velocityError.angVel
                            )
                        ), 2
                    )
                )
                drive.setMotorPowers(
                    feedforward.update(wheelVelocities.rightFront),
                    feedforward.update(wheelVelocities.rightBack),
                    feedforward.update(wheelVelocities.leftFront),
                    feedforward.update(wheelVelocities.leftBack)
                )
            }
            .finished { drive.defaultEndCondition(pose) }
            .build()
    }

    override fun lineTo(pose: Pose2d, end: Supplier<Boolean>): Command {
        return Command.builder()
            .init {
                axialPIDController.reset()
                lateralPIDController.reset()
                headingPIDController.reset()
            }
            .update {
                drive.update()
                val currentPose = drive.pose.value()
                val velocityError = -drive.pose.velocity().value()


                val wheelVelocities: WheelVelocities<Time> = kinematics.inverse(
                    PoseVelocity2dDual.constant(
                        PoseVelocity2d(
                            Vector2d(
                                axialPIDController.update(
                                    pose.x,
                                    currentPose.x,
                                    velocityError.linearVel.x
                                ),
                                lateralPIDController.update(
                                    pose.y,
                                    currentPose.y,
                                    velocityError.linearVel.y
                                )
                            ),
                            headingPIDController.update(
                                pose.heading.log(),
                                currentPose.heading.log(),
                                velocityError.angVel
                            )
                        ), 2
                    )
                )
                drive.setMotorPowers(
                    feedforward.update(wheelVelocities.rightFront),
                    feedforward.update(wheelVelocities.rightBack),
                    feedforward.update(wheelVelocities.leftFront),
                    feedforward.update(wheelVelocities.leftBack)
                )
            }
            .finished(end)
            .build()
    }
}
