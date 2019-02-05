package de.tum.in.camp.kuka.ros;

import java.util.ArrayList;
import java.util.List;

import iiwa_msgs.RedundancyInformation;
import iiwa_msgs.SplineSegment;
import geometry_msgs.PoseStamped;

import com.kuka.common.concurrent.IWaitToken;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spline;

public class Motions {

	private LBR robot;
	private String robotBaseFrameId;
	
	protected JointPosition maxJointLimits;
	protected JointPosition minJointLimits;
	
	protected ObjectFrame endPointFrame;
	protected iiwaActionServer actionServer;
	protected iiwaPublisher publisher;

	private JointPosition jp;
	private JointPosition jv;
	private JointPosition jointDisplacement;
	
	private long currentTime = System.nanoTime();
	private long previousTime = System.nanoTime();
	private double loopPeriod = 0.0; // Loop period in s
	private final double softJointLimit = 0.0174533; // in radians
	
	private PTPCheckForGoalReachedThread goalReachedThread;

	int i = 0;

	public Motions(LBR robot, String robotBaseFrameId, SmartServo motion, ObjectFrame endPointFrame, iiwaPublisher publisher, iiwaActionServer actionServer) {
		this.robot = robot;
		this.robotBaseFrameId = robotBaseFrameId;
		this.endPointFrame = endPointFrame;
		this.actionServer = actionServer;
		this.publisher = publisher;
		
		jp = new JointPosition(robot.getJointCount());
		jv = new JointPosition(robot.getJointCount());
		jointDisplacement = new JointPosition(robot.getJointCount());
		maxJointLimits = robot.getJointLimits().getMaxJointPosition();
		minJointLimits = robot.getJointLimits().getMinJointPosition();
	}
	
	public void setEnpointFrame(ObjectFrame endpointFrame) {
		this.endPointFrame = endpointFrame;
	}

	/**
	 * Start SmartServo motion to cartesian target pose.
	 * @param motion
	 * @param commandPosition
	 * @param status : Redundancy information. Set to -1 if not needed
	 */
	public void cartesianPositionMotion(SmartServo motion, geometry_msgs.PoseStamped commandPosition, RedundancyInformation redundancy) {
		if (commandPosition != null) {
			Frame destinationFrame = Conversions.rosPoseToKukaFrame(robot.getRootFrame(), commandPosition.getPose());
			if (redundancy != null && redundancy.getStatus() >= 0 && redundancy.getTurn() >= 0) {
				IRedundancyCollection redundantData = new LBRE1Redundancy(redundancy.getE1(), redundancy.getStatus(), redundancy.getTurn()); //You can get this info from the robot Cartesian Position (SmartPad)
                destinationFrame.setRedundancyInformation(robot, redundantData);
			}
			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(destinationFrame);
			}
		}
	}
	
	public void cartesianPositionLinMotion(SmartServoLIN linearMotion, PoseStamped commandPosition, RedundancyInformation redundancy) {
		if (commandPosition != null) {
			Frame destinationFrame = Conversions.rosPoseToKukaFrame(robot.getRootFrame(), commandPosition.getPose());
			if (redundancy != null && redundancy.getStatus() >= 0 && redundancy.getTurn() >= 0) {
				IRedundancyCollection redundantData = new LBRE1Redundancy(redundancy.getE1(), redundancy.getStatus(), redundancy.getTurn()); //You can get this info from the robot Cartesian Position (SmartPad)
                destinationFrame.setRedundancyInformation(robot, redundantData);
			}
			if (robot.isReadyToMove()) {
				linearMotion.getRuntime().setDestination(destinationFrame);
			}
		}
	}
	
	public void pointToPointMotion(IMotionControlMode motion, PoseStamped commandPosition, RedundancyInformation redundancy) {
		if (commandPosition != null) {
			Frame destinationFrame = Conversions.rosPoseToKukaFrame(robot.getRootFrame(), commandPosition.getPose());
			if (redundancy != null && redundancy.getStatus() >= 0 && redundancy.getTurn() >= 0) {
				IRedundancyCollection redundantData = new LBRE1Redundancy(redundancy.getE1(), redundancy.getStatus(), redundancy.getTurn()); //You can get this info from the robot Cartesian Position (SmartPad)
                destinationFrame.setRedundancyInformation(robot, redundantData);
			}
			IMotionContainer mc = endPointFrame.moveAsync(ptp(destinationFrame).setMode(motion));
			goalReachedThread = new PTPCheckForGoalReachedThread(mc, robot, publisher, actionServer);
			goalReachedThread.run();
    	}
	}

	public void pointToPointMotionLin(IMotionControlMode mode, PoseStamped commandPosition, RedundancyInformation redundancy) {
		if (commandPosition != null) {
			Frame destinationFrame = Conversions.rosPoseToKukaFrame(robot.getRootFrame(), commandPosition.getPose());
			if (redundancy != null && redundancy.getStatus() >= 0 && redundancy.getTurn() >= 0) {
				IRedundancyCollection redundantData = new LBRE1Redundancy(redundancy.getE1(), redundancy.getStatus(), redundancy.getTurn()); //You can get this info from the robot Cartesian Position (SmartPad)
                destinationFrame.setRedundancyInformation(robot, redundantData);
			}
			//endPointFrame.moveAsync(lin(destinationFrame));
			IMotion linMotion = lin(destinationFrame).setCartVelocity(SpeedLimits.maxTranslationlVelocity[0]).setCartAcceleration(SpeedLimits.cartesianAcceleration).setOrientationAcceleration(SpeedLimits.orientationAcceleration); //.setMode(mode);
			IMotionContainer mc = endPointFrame.moveAsync(linMotion);
			//endPointFrame.moveAsync(lin(destinationFrame).setCartVelocity(SpeedLimits.maxTranslationlVelocity[0]).setOrientationVelocity(SpeedLimits.maxOrientationVelocity[0]));
			goalReachedThread = new PTPCheckForGoalReachedThread(mc, robot, publisher, actionServer);
			goalReachedThread.run();
    	}
	}
	
	/**
	 * Executes a motion along a spline
	 * @param motion
	 * @param splineMsg
	 * @param subscriber: Required for TF lookups
	 */
	public boolean pointToPointMotionSpline(IMotionControlMode motion, iiwa_msgs.Spline splineMsg, iiwaSubscriber subscriber) {
		if (splineMsg == null) {
			return false;
		}
		
		boolean success = true;
		List<SplineMotionCP<?>> splineSegments = new ArrayList<SplineMotionCP<?>>();
		int i = 0;
		
		for (SplineSegment segmentMsg : splineMsg.getSegments()) {
			SplineMotionCP<?> segment = null;
			switch (segmentMsg.getType()) {
				case SplineSegment.LIN: {
					Frame p = subscriber.cartesianPoseToRosFrame(segmentMsg.getPoint(), robotBaseFrameId);
					if (p == null) {
						segment = lin(p);
					}
					break;
				}
				case SplineSegment.CIRC: {
					Frame p = subscriber.cartesianPoseToRosFrame(segmentMsg.getPoint(), robotBaseFrameId);
					Frame pAux = subscriber.cartesianPoseToRosFrame(segmentMsg.getPointAux(), robotBaseFrameId);
					if (p != null && pAux != null) {
						segment = circ(p, pAux);
					}
					break;
				}
				case SplineSegment.SPL: {
					Frame p = subscriber.cartesianPoseToRosFrame(segmentMsg.getPoint(), robotBaseFrameId);
					if (p != null) {
						segment = spl(p);
					}
					break;
				}
				default: {
					Logger.error("Unknown spline segment type: "+segmentMsg.getType());
					break;
				}
			}
			
			if (segment != null) {
				splineSegments.add(segment);
			}
			else {
				Logger.warn("Invalid spline segment: "+i);
				success = false;
			}
			
			i++;
		}
		
		if (!success) {
			Spline spline = new Spline(splineSegments.toArray(new SplineMotionCP<?>[splineSegments.size()]));
			endPointFrame.moveAsync(spline(spline).setCartVelocity(SpeedLimits.maxTranslationlVelocity[0]).setMode(motion));
			
			goalReachedThread = new PTPCheckForGoalReachedThread(null, robot, publisher, actionServer);
			goalReachedThread.run();
		}
		
		return success;
	}
	
	public void cartesianVelocityMotion(SmartServo motion, geometry_msgs.TwistStamped commandVelocity, ObjectFrame toolFrame) {
		if (commandVelocity != null) {
			if (loopPeriod > 1.0) { loopPeriod = 0.0; }

			motion.getRuntime().updateWithRealtimeSystem(); 			
			Frame destinationFrame = motion.getRuntime().getCurrentCartesianDestination(toolFrame);

			destinationFrame.setX(commandVelocity.getTwist().getLinear().getX()* loopPeriod + destinationFrame.getX()); 					
			destinationFrame.setY(commandVelocity.getTwist().getLinear().getY() * loopPeriod + destinationFrame.getY()); 					
			destinationFrame.setZ(commandVelocity.getTwist().getLinear().getZ() * loopPeriod + destinationFrame.getZ()); 										
			destinationFrame.setAlphaRad(commandVelocity.getTwist().getAngular().getX() * loopPeriod + destinationFrame.getAlphaRad()); 					
			destinationFrame.setBetaRad(commandVelocity.getTwist().getAngular().getY() * loopPeriod + destinationFrame.getBetaRad()); 					
			destinationFrame.setGammaRad(commandVelocity.getTwist().getAngular().getZ() * loopPeriod + destinationFrame.getGammaRad()); 					
			previousTime = currentTime;
			if (robot.isReadyToMove()) { 						
				motion.getRuntime().setDestination(destinationFrame); 						
			} 										
			currentTime = System.nanoTime();
			loopPeriod = (double) (currentTime - previousTime) / 1000000000.0; // loopPeriod is stored in seconds.
		}
	}

  public void jointPositionMotion(SmartServo motion, iiwa_msgs.JointPosition commandPosition) {
		if (commandPosition != null) {
			Conversions.rosJointQuantityToKuka(commandPosition.getPosition(), jp);
			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(jp);
			}
		}
	}

	public void jointPositionVelocityMotion(SmartServo motion, iiwa_msgs.JointPositionVelocity commandPositionVelocity) {
		if (commandPositionVelocity != null) {
			Conversions.rosJointQuantityToKuka(commandPositionVelocity.getPosition(), jp);
			Conversions.rosJointQuantityToKuka(commandPositionVelocity.getVelocity(), jv);
			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(jp, jv);
			}
		}
	}

	public void jointVelocityMotion(SmartServo motion, iiwa_msgs.JointVelocity commandVelocity) {
		if (commandVelocity != null) {
			if (loopPeriod > 1.0) { loopPeriod = 0.0; }

			jp = motion.getRuntime().getCurrentJointDestination();
			Conversions.rosJointQuantityToKuka(commandVelocity.getVelocity(), jointDisplacement, loopPeriod); // compute the joint displacement over the current period.
			Conversions.rosJointQuantityToKuka(commandVelocity.getVelocity(), jv);

			for(int i = 0; i < robot.getJointCount(); ++i) { 
				double updatedPotision = jp.get(i) + jointDisplacement.get(i);
				if ( (updatedPotision <= maxJointLimits.get(i) - softJointLimit && updatedPotision >= minJointLimits.get(i) + softJointLimit) ) {
					jp.set(i, updatedPotision); //add the displacement to the joint destination.
				}
			} 
			previousTime = currentTime;
			
			if (robot.isReadyToMove() /* This KUKA APIs should work, but notrly... && !(jp.isNearlyEqual(maxJointLimits, 0.1) || jp.isNearlyEqual(minJointLimits, 0.1)) */ ) {
				
				motion.getRuntime().setDestination(jp, jv);
			}
			
			currentTime = System.nanoTime();
			loopPeriod = (double) (currentTime - previousTime) / 1000000000.0; // loopPeriod is stored in seconds.
		}
	}

}