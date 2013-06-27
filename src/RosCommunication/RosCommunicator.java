package RosCommunication;

import java.util.LinkedList;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class RosCommunicator {

	NodeMainExecutor nodeMainExecutor;
	
	SubscriberNode subNode;
	

	public RosCommunicator(String RosMasterIp, LinkedList<ISubscriberInfo> info) {

		System.out.println("creating RosCommunicator");
		nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

		System.out.println("ROS Master IP: " + RosMasterIp);
		java.net.URI muri = java.net.URI.create(RosMasterIp);

		NodeConfiguration nodeConf = NodeConfiguration.newPublic("127.0.0.1",
				muri);
		nodeConf.setNodeName("zusmoro/RosCommunicator");
		
		subNode = new SubscriberNode(info);

		nodeMainExecutor.execute(subNode, nodeConf);
	}
	
	/**
	 * Returns an Object, representing the Ros-message that is associated with this sensorName.
	 * To use the result, try to carst the returned Object to the expected TopicType.
	 * 
	 * E.g. (std_msgs.Int32)getSensorMsg("mySensor")
	 * 	
	 * @param sensorName you would like to get the last received message from
	 * @return last received message of the associated sensor
	 */
	public Object getSensorMsg(String sensorName){
		return subNode.getSensorMsg(sensorName);
	}

}
