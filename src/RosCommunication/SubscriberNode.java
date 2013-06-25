package RosCommunication;

import java.util.HashMap;
import java.util.LinkedList;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class SubscriberNode extends AbstractNodeMain {

	private LinkedList<ISubscriberInfo> info;
	private HashMap<String, Integer> sensorValues;

	public SubscriberNode(LinkedList<ISubscriberInfo> info) {
		this.info = info;
		sensorValues = new HashMap<>();
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		final Publisher<std_msgs.Int32> publisher = connectedNode.newPublisher(
				"topic/IR0", std_msgs.Int32._TYPE);

		createSubscribers(connectedNode);
		
		// This Loop will be canceled automatically when the node shuts down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				std_msgs.Int32 zahl = connectedNode.getTopicMessageFactory()
						.newFromType(std_msgs.Int32._TYPE);
				zahl.setData(sequenceNumber);
				publisher.publish(zahl);
				sequenceNumber++;
				Thread.sleep(1000);
			}
		});
	}

	private void createSubscribers(ConnectedNode connectedNode) {
		for (ISubscriberInfo sub : info) {
			// TODO check sub.getTopicType and create correct Subscriber
			Subscriber<std_msgs.Int32> subscriber = connectedNode
					.newSubscriber(sub.getTopic(), std_msgs.Int32._TYPE);

			final String sensorName = sub.getName();
			subscriber
					.addMessageListener(new MessageListener<std_msgs.Int32>() {

						@Override
						public void onNewMessage(std_msgs.Int32 arg0) {
							sensorValues.put(sensorName, arg0.getData());
							System.out.println(sensorName + " = "
									+ arg0.getData());
						}
					});
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
