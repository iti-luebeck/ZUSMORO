package RosCommunication;

import java.util.LinkedList;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class RosCommunicator {

	NodeMainExecutor nodeMainExecutor;

	public RosCommunicator(String RosMasterIp, LinkedList<ISubscriberInfo> info) {

		System.out.println("creating RosCommunicator");
		nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

		System.out.println("ROS Master IP: " + RosMasterIp);
		java.net.URI muri = java.net.URI.create(RosMasterIp);

		NodeConfiguration nodeConf = NodeConfiguration.newPublic("127.0.0.1",
				muri);
		nodeConf.setNodeName("zusmoro/RosCommunicator");
		
		SubscriberNode systemNode = new SubscriberNode(info);

		nodeMainExecutor.execute(systemNode, nodeConf);
	}
	
	

}
