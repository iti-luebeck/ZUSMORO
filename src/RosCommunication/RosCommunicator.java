package RosCommunication;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedList;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class RosCommunicator {

	private NodeMainExecutor nodeMainExecutor;
	
	private SubscriberNode subNode;
	
	private LinkedList<ActionListener> listeners;
	private String rosMasterIP;

	public RosCommunicator(String rosMasterIp, LinkedList<ISubscriberInfo> info) {

		this.rosMasterIP = rosMasterIp;
		
		System.out.println("creating RosCommunicator");
		
		listeners = new LinkedList<>();
		
		nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

		System.out.println("ROS Master IP: " + rosMasterIp);		
		
		subNode = new SubscriberNode(info, this);
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
	
	public void addActionListener(ActionListener listener){
		listeners.add(listener);
		System.out.println("Listeneranzahl: "+listeners.size());
	}
	
	void notifyListeners(ActionEvent e){
		for(ActionListener l : listeners){
			l.actionPerformed(e);
		}
	}
	
	public void removeListener(ActionListener listener){
		listeners.remove(listener);
	}
	
	public void startCommunication(){
		java.net.URI muri = java.net.URI.create(rosMasterIP);
		NodeConfiguration nodeConf = NodeConfiguration.newPublic("127.0.0.1",
				muri);
		nodeConf.setNodeName("zusmoro/RosCommunicator");
		
		nodeMainExecutor.execute(subNode, nodeConf);
	}
	
	public void shutdown(){
		nodeMainExecutor.shutdown();
	}

}
