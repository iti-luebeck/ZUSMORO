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
package model.onboard;

import java.io.IOException;
import java.util.Observable;
import java.util.Observer;

import robots.epuck.Communicator;
import view.MainFrame;

public class BTLayer implements Observer {

	private String lastMsgRec;

	private String lastMsgSent;

	private final String VERSION = "Vzusmoro1.2.0";

	private Communicator comm;
	private TransmissionJob job;

	boolean started;
	boolean ok_version;
	boolean start_jobs;
	
	private int notif = 0;
	
	private int initCounter = 0;

	/*public BTLayer(String portName, TransmissionJob job) {
		comm = new Communicator(portName);
		comm.addObserver(this);
		lastMsgSent = null;
		lastMsgRec = "";
		this.job = job;
		started = false;
	}*/

	public BTLayer(Communicator comm, TransmissionJob job) {
		System.out.println("CREATING BT!!!!");
		this.comm = comm;
		comm.addObserver(this);
		lastMsgSent = null;
		lastMsgRec = "";
		this.job = job;

		started = false;
		ok_version = false;
		start_jobs = false;
	}
	
	public void deattachObserver(){
		if(comm != null){
			comm.deleteObserver(this);
		}
	}

	@Override
	public void update(Observable o, Object arg) {
		System.out.println("Initcounter: " + initCounter);
		lastMsgRec = (String) arg + "\r";
		System.out.println("Ret:" + lastMsgRec);
		if (arg.equals("vx") && !ok_version) {//arg.equals(VERSION)
			ok_version = true;
			send();
			return;
		}
		
		if(!ok_version && initCounter < 5){//version hat nicht gelappt, nochmal. aber maximal 5 mal
			sendingInit();
			return;
		}
		/*if (arg.equals("vx")) {
			start_jobs = true;
		}*/
		started = true;

//		lastMsgRec = (String) arg + "\r";
//		System.out.println("Ret:" + lastMsgRec);
		if(lastMsgSent != null){
			if (lastMsgSent.equals(lastMsgRec)) {
				job.removeFirst();
				send();
			} else {
				//check if job is complete and if last unit has been transfered correctly
				if (!job.isComplete() /*&& !job.check(lastMsgRec, lastMsgSent)*/) {
					send();
				}
	
			}
		}

	}

	private void send() {
		try {

			if (!job.isComplete()) {
				MainFrame.toolPanel.enableStart(false);
				MainFrame.toolPanel.enableTransmit(false);
				lastMsgSent = job.getFirst() + "\r";
				System.out.println("Sending:" + lastMsgSent);
				comm.writeToStream(lastMsgSent);
                        } else {//wenn job nicht completed wird damm kann man zwar noch resenden aber observer wird nicht gel�scht!

				comm.deleteObserver(this);
				MainFrame.toolPanel.enableStart(true);
				MainFrame.toolPanel.enableTransmit(true);
				MainFrame.toolPanel.enableDebug(true);
			}

		} catch (IOException e) {
			System.out.println("Error Sending:" + lastMsgSent);
			e.printStackTrace();
		}

	}
	
	private void sendingInit(){
		System.out.println("Sending inital VX");
		try {
			initCounter++;
			comm.writeToStream("vx\r");
			//Thread.sleep(1000);//200
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}//catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//}
	}

	public boolean start() {
		/*for (int i = 0; i < 5 && !ok_version && !start_jobs; i++) {
			try {
				System.out.println("Sending inital VX");
				comm.writeToStream("vx\r");
				Thread.sleep(200);//200
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return false;
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				return false;
			}

		}*/
		
		sendingInit();
		/*try{
			Thread.sleep(1000);
		}
		catch (InterruptedException e) {
			// TODO Auto-generated catch block
		}*/
		
		if(initCounter > 5){
			return false;
		}
		
		/*if(ok_version){
			send();
		}*/
		
//		return ok_version;
		return true;
	}

}
