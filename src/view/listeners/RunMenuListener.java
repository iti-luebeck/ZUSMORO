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
package view.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.TreeMap;
import view.MainFrame;

public class RunMenuListener implements ActionListener {

	private interface Method {
		void doEvent(ActionEvent e);
	}

	private Method connect = new Method() {
		public void doEvent(ActionEvent e) {
			String connectTo = MainFrame.robot.getLastConnectedTo();
			boolean connected = MainFrame.robot.connect(connectTo);
			MainFrame.toolPanel.setConnected(connected);
			MainFrame.toolPanel.enableStart(connected);
			MainFrame.toolPanel.enableStop(connected);
			MainFrame.toolPanel.enableDebug(connected);
		}
	};

	private Method debug = new Method() {// TODO auslagern, Für beep smach
											// viewer starten
		@Override
		public void doEvent(ActionEvent e) {
			MainFrame.robot.debug();
		}
	};

	private Method disconnect = new Method() {
		public void doEvent(ActionEvent e) {
			MainFrame.robot.disconnect();
			MainFrame.toolPanel.setConnected(false);
		};
	};

	TreeMap<String, Method> methods;

	private Method run = new Method() {
		public void doEvent(ActionEvent e) {
			MainFrame.robot.play();
			MainFrame.toolPanel.enableStop(true);
			MainFrame.toolPanel.enableDebug(true);
		}
	};

	private Method stop = new Method() {
		public void doEvent(ActionEvent e) {
			MainFrame.robot.stop();
		};
	};

	private Method transmit = new Method() {
		public void doEvent(ActionEvent e) {
			if (!MainFrame.robot.transmit()) {
				MainFrame
						.showErrInfo(
								"Die generierte Datei konnte nicht übertragen werden. Eventuell hilft ein neuer Verbindungsaufbau.",
								"Übertragungsfehler");
			}
		}
	};

	public RunMenuListener() {
		methods = new TreeMap<String, Method>();
		methods.put("run", run);
		methods.put("stop", stop);
		methods.put("connect", connect);
		methods.put("disconnect", disconnect);
		methods.put("transmit", transmit);
		methods.put("debug", debug);
	}

	public void actionPerformed(ActionEvent e) {
		Method m = methods.get(e.getActionCommand());
		try {
			m.doEvent(e);
		} catch (NullPointerException ex) {
			System.out.println("ActionCommand nicht gefunden:"
					+ e.getActionCommand());
			ex.printStackTrace();

		}

	}

}
