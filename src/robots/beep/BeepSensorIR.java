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
package robots.beep;

import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import RosCommunication.ISubscriberInfo;

import model.bool.Variable.Operator;

import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepSensorIR implements ISmachableSensor, ISubscriberInfo {

	private String name;
	private String topic;
	private final int irIndex;
	private final String topicType = beep_msgs.IR._TYPE;

	public BeepSensorIR(String name, String topic, int irIndex) {
		this.name = name;
		this.topic = topic;
		this.irIndex = irIndex;
	}

	public BeepSensorIR() {
		name = null;
		topic = null;
		irIndex = 0;
	}

	@Override
	public String getTopic() {
		return topic;
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public String getTransitionCondition(String op, int compVal) {
		return getValueIdentifier() + op + compVal;
	}

	@Override
	public HashSet<String> getImports() {
		String temp[] = topicType.split("/");
		HashSet<String> res = new HashSet<String>();
		res.add("from " + temp[0] + ".msg import " + temp[1]);
		return res;
	}

	@Override
	public String getCallback() {
		String res = "";
		res += "def ir_cb(msg):\n";
		res += "\tglobal ir\n";
		res += "\tir = msg.ir\n";
		return res;
	}

	@Override
	public String getSubscriberSetup() {
		String res = "";
		res = "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
				+ ", ir_cb)\n";
		return res;
	}

	@Override
	public String getValueIdentifier() {
		return "ir[" + irIndex + "]";
	}

	@Override
	public String getTopicType() {
		return topicType;
	}

	@Override
	public String getIdentifierInit() {
		return "ir = [0, 0, 0, 0, 0, 0, 0, 0]";
	}

	@Override
	public String getGlobalIdentifier() {
		return "ir";
	}

	@Override
	public String[] onShutDown() {
		// TODO Alle Aus, Hardware noch nicht bereit
		return new String[0];
	}

}
