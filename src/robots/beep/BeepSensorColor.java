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

import java.awt.Color;
import java.util.HashSet;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;

import RosCommunication.ISubscriberInfo;

import model.bool.Variable.Operator;
import smachGenerator.ISmachableSensor;

@XmlAccessorType(XmlAccessType.FIELD)
public class BeepSensorColor implements ISmachableSensor, ISubscriberInfo {

	private String name;
	private String topic;
	private final String topicType = beep_msgs.Color_sensors._TYPE;
	private final int sensorIndex;

	public BeepSensorColor(String name, String topic, int sensorIndex) {
		this.name = name;
		this.topic = topic;
		this.sensorIndex = sensorIndex;
	}

	public BeepSensorColor() {
		name = null;
		topic = null;
		sensorIndex = 0;
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
	public String getTopicType() {
		return topicType;
	}

	@Override
	public String getTransitionCondition(String op, int compVal) {
		Color col = new Color(compVal);
		float[] hsbCol = Color.RGBtoHSB(col.getRed(), col.getGreen(),
				col.getBlue(), null);

		return getValueIdentifier() + ">" + (hsbCol[0] - 0.1 + 1) % 1 + " and "
				+ getValueIdentifier() + "<" + (hsbCol[0] + 0.1) % 1;
	}

	@Override
	public HashSet<String> getImports() {
		HashSet<String> res = new HashSet<String>();
		res.add("from " + topicType.split("/")[0] + ".msg import "
				+ topicType.split("/")[1]);
		res.add("import colorsys");
		return res;
	}

	@Override
	public String getCallback() {
		String res = "";
		res += "def color_cb(msg):\n";
		res += "\tglobal colorSensor\n";
		res += "\tfor (i, sensor) in enumerate(msg.sensors):\n";
		res += "\t\tcolorSensor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]\n";
		return res;
	}

	@Override
	public String getSubscriberSetup() {
		String res = "";
		res += "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
				+ ", color_cb)\n";
		return res;
	}

	@Override
	public String getValueIdentifier() {
		return "colorSensor[" + sensorIndex + "]";
	}

	public String getIdentifierInit() {
		return "colorSensor = [0, 0, 0]";
	}

	@Override
	public String getGlobalIdentifier() {
		return "colorSensor";
	}

	@Override
	public String[] onShutDown() {
		// TODO Auto-generated method stub
		return new String[0];
	}

}
