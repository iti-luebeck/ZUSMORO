package RosCommunication;

import java.util.HashMap;
import java.util.LinkedList;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

public class SubscriberNode extends AbstractNodeMain {

	private RosCommunicator rosComm;
	private LinkedList<ISubscriberInfo> info;
	private HashMap<String, Object> sensorValues;

	public SubscriberNode(LinkedList<ISubscriberInfo> info,
			RosCommunicator rosComm) {
		this.rosComm = rosComm;
		this.info = info;
		sensorValues = new HashMap<>();
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		createSubscribers(connectedNode);

		// final Publisher<std_msgs.String> publisher = connectedNode
		// .newPublisher("topic/IR0", std_msgs.String._TYPE);
		//
		//
		//
		// // This Loop will be canceled automatically when the node shuts down.
		// connectedNode.executeCancellableLoop(new CancellableLoop() {
		// private int sequenceNumber;
		//
		// @Override
		// protected void setup() {
		// sequenceNumber = 0;
		// }
		//
		// @Override
		// protected void loop() throws InterruptedException {
		// std_msgs.String zahl = connectedNode.getTopicMessageFactory()
		// .newFromType(std_msgs.String._TYPE);
		// zahl.setData("hallo");
		// publisher.publish(zahl);
		// sequenceNumber++;
		// Thread.sleep(1000);
		// }
		// });
	}
	
	@Override
	public void onShutdown(Node arg0) {
		super.onShutdown(arg0);
		System.out.println("shutting down");
	}

	private void createSubscribers(ConnectedNode connectedNode) {
		for (ISubscriberInfo sub : info) {
			Subscriber<Object> subscriber = connectedNode.newSubscriber(
					sub.getTopic(), sub.getTopicType());

			final String sensorName = sub.getName();
			subscriber.addMessageListener(new MessageListener<Object>() {

				@Override
				public void onNewMessage(Object arg0) {
					sensorValues.put(sensorName, arg0);
					rosComm.notifyListeners(null);
				}
			});
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	public Object getSensorMsg(String sensorName) {
		return sensorValues.get(sensorName);
	}

}
