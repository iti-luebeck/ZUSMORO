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

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import ch.ethz.ssh2.Connection;
import ch.ethz.ssh2.SCPClient;
import ch.ethz.ssh2.Session;
import ch.ethz.ssh2.StreamGobbler;

import smachGenerator.SmachableActuators;
import smachGenerator.SmachableSensors;
import view.AbstractStatePanel;
import view.AbstractTransitionPanel;
import model.AbstractRobot;
import model.Action;
import model.State;
import model.Transition;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class BeepRobot extends AbstractRobot {

	@XmlElement(name = "IR_Sensor")
	List<BeepIRSensor> sensorsIR = new ArrayList<BeepIRSensor>();

	@XmlElement(name = "Color_Sensor")
	List<BeepColorSensor> sensorsCol = new ArrayList<BeepColorSensor>();

	@XmlElement(name = "actuator")
	List<BeepActuator> actuators = new ArrayList<BeepActuator>();

	Connection conn;
	Session sess;

	PrintWriter piCommand;

	public BeepRobot() {
		// Define default Beep sensors
		sensorsIR.add(new BeepIRSensor("IR0", "topic/IR0", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR1", "topic/IR1", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR2", "topic/IR2", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR3", "topic/IR3", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR4", "topic/IR4", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR5", "topic/IR5", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR6", "topic/IR6", "Int8",
				"std_msgs.msg", "data"));
		sensorsIR.add(new BeepIRSensor("IR7", "topic/IR7", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR0", "topic/UIR0", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR1", "topic/UIR2", "Int8",
				"std_msgs.msg", "data"));
		sensorsCol.add(new BeepColorSensor("UIR2", "topic/UIR3", "Int8",
				"std_msgs.msg", "data"));

		// Define default Beep actuators
		// actuators.add(new BeepActuator("MOTOR1", "topic/motors", "Motors",
		// "beep.msg", "links"));
		// actuators.add(new BeepActuator("MOTOR2", "topic/motors", "Motors",
		// "beep.msg", "rechts"));
		actuators.add(new BeepActuator("LED1", "topic/LED1", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED2", "topic/LED2", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED3", "topic/LED3", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED4", "topic/LED4", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED5", "topic/LED5", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED6", "topic/LED6", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED7", "topic/LED7", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED0", "topic/LED0", "Int8",
				"std_msgs.msg", "data"));
		actuators.add(new BeepActuator("LED8", "topic/LED8", "Int8",
				"std_msgs.msg", "data"));
		BeepActuator beep = new BeepActuator("BEEP", "topic/beep", "Int8",
				"std_msgs.msg", "data");
		actuators.add(beep);
	}

	public SmachableSensors getSensors() {
		SmachableSensors sen = new SmachableSensors();
		sen.addAll(sensorsIR);
		sen.addAll(sensorsCol);
		return sen;
	}

	public SmachableActuators getActuators() {
		SmachableActuators act = new SmachableActuators();
		act.addAll(actuators);
		return act;
	}

	@Override
	public boolean connect(String connectTo) {
		conn = new Connection(connectTo);//beep: "141.83.158.207"
		try {
			//connect and authorize
			conn.connect();
			conn.authenticateWithPassword("pi", "beep");

			//start a compatible shell
			sess = conn.openSession();
			sess.requestDumbPTY();
			sess.startShell();
			piCommand = new PrintWriter(sess.getStdin(), true);
			//echo to recognize when shell is ready to use
			piCommand.println("echo 'ready to start'");

			//print output and block until shell is "ready to start"
			InputStream stdout = new StreamGobbler(sess.getStdout());
			BufferedReader br = new BufferedReader(
					new InputStreamReader(stdout));
			while (true) {
				String line = br.readLine();
				if (line == null) {
					break;
				}
				System.out.println(line);
				if (line.equals("ready to start")) {
					break;
				}
			}

			br.close();
			stdout.close();

			return true;
		} catch (Exception e) {
			e.printStackTrace();
		}
		return false;
	}

	@Override
	public void disconnect() {
		piCommand.close();
		piCommand = null;
		sess.close();
		sess = null;
		conn.close();
		conn = null;
	}

	@Override
	public int getVariableValue(String variable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void executeActions(ArrayList<Action> actions) throws IOException {
		// TODO Auto-generated method stub

	}

	@Override
	public void updateSensors() throws IOException {
		// TODO Auto-generated method stub

	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub

	}
	
	@Override
	public void transmit() {
		SCPClient client = new SCPClient(conn);
		try {
			client.put("test.py", "~/Beep/Software/catkin_ws/src/beep_imu");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void play() {
		if (conn != null && sess != null && piCommand != null) {
			piCommand.println("mkdir -p ~/log");
			piCommand
					.println("nohup roscore 2> ~/log/roscore-err.log 1> ~/log/roscore-out.log &");
			piCommand
					.println("nohup rosrun beep_imu ir_distance.py 2> ~/log/ir_distance-err.log 1> ~/log/ir_distance-out.log");
		}
	}

	@Override
	public int getDesiredAdditionalTimeout() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public AbstractStatePanel getStatePanel(State state) {
		return new BeepStatePanel(state);
	}

	@Override
	public AbstractTransitionPanel getTransitionPanel(Transition trans) {
		return new BeepTransitionPanel(trans);
	}

	@Override
	public int getUnAcknowledgedCmds() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Stores a given {@link BeepRobot} as a xml file to the given path
	 * 
	 * @param robot
	 *            that will be stored
	 * @param file
	 *            where the xml file will be stored
	 * @return <code>null</code> if no error occurred else an error information.
	 */
	public static String saveBeepRobot(BeepRobot robot, File file) {
		try {
			JAXBContext context = JAXBContext.newInstance(BeepRobot.class);
			Marshaller m = context.createMarshaller();
			m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
			file.setWritable(true);
			Path toPath = file.toPath();
			try {
				BufferedWriter newBufferedWriter = Files.newBufferedWriter(
						toPath, Charset.defaultCharset());
				m.marshal(robot, newBufferedWriter);
				newBufferedWriter.flush();
			} catch (IOException ex) {
				return "Can't write File: " + file.getAbsolutePath()
						+ " . No Write Access";
			}
		} catch (JAXBException ex) {
			ex.printStackTrace();
		}
		return null;
	}

	/**
	 * Creates a {@link BeepRobot} from a given xml file. The expected pattern
	 * is: beepRobot(Device*) Device(topic, name, objectInMessage, topicType,
	 * topicPackage)
	 * 
	 * @param fileName
	 *            the xml is stored
	 * @return a {@link BeepRobot}
	 */
	public static BeepRobot loadBeepRobot(String fileName) {
		try {
			if (!fileName.endsWith(".xml")) {
				fileName += ".xml";
			}
			File file = new File(fileName);
			if (file.exists()) {
				JAXBContext context = JAXBContext.newInstance(BeepRobot.class);
				Unmarshaller u = context.createUnmarshaller();
				BeepRobot robot = (BeepRobot) u.unmarshal(file);
				return robot;
			} else {
				return null;
			}
		} catch (JAXBException ex) {
			ex.printStackTrace();
		}
		return null;
	}

	@Override
	public String getRobotName() {
		return "Beep Robot";
	}

}
