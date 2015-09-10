/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package RosCommunication;

import java.awt.event.ActionEvent;
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
				public void onNewMessage(Object msg) {
					sensorValues.put(sensorName, msg);
					ActionEvent event = new ActionEvent(msg, 0, sensorName);
					rosComm.notifyListeners(event);
				}
			}, 15);
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return null;
	}

	public Object getSensorMsg(String sensorName) {
		return sensorValues.get(sensorName);
	}

}
