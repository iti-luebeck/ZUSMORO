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
package robots.epuck;

import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import gnu.io.UnsupportedCommOperationException;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Observable;
import java.util.TooManyListenersException;
import java.util.Scanner;

import javax.swing.JOptionPane;

import model.Automat;
import view.MainFrame;

public class Communicator extends Observable implements SerialPortEventListener {
	private CommPortIdentifier portIdentifier;
	private SerialPort serialPort;
	private InputStream inStream;
	private OutputStream outStream;
	private WriteThread writeThread;
	private volatile boolean isConnected = false;
	private final Object writeSignal = new Object();
	private final static int eva = 100;
	private final static int stringBuilderLength = 100;
	private StringBuilder readBuffer = new StringBuilder(stringBuilderLength);
	private int serialCounter = 0;
	//private Scanner in;
	private BufferedReader d;

	public Communicator(String portName) {
		if (MainFrame.DEBUG) {
		System.out.println("New Communicator for: "+portName);
		}
		try {
			portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
			serialPort = (SerialPort) portIdentifier.open("Communicator", 2000);
			inStream = serialPort.getInputStream();
			//in  = new Scanner( inStream );
			d = new BufferedReader(new InputStreamReader(inStream));
			outStream = serialPort.getOutputStream();
			initSerialPort();
			isConnected = true;
			if (MainFrame.DEBUG) {
			System.out.println("Communicator connected to: "+portName);
			}
		} catch (NoSuchPortException e) {
			JOptionPane.showMessageDialog(MainFrame.mainFrame, "Der angegebene Port " + portName
					+ " konnte nicht gefunden werden.", "Fehler beim Verbindungsaufbau", JOptionPane.WARNING_MESSAGE);
			return;
		} catch (PortInUseException e) {
			JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Der angegebene Port " + portName
					+ " konnte nicht geöffnet werden.<br>" +
							"Ist die Verbindung zum Roboter zuvor ungewollt<br>" +
							"unterbrochen worden und verwendet kein anderes<br>" +
							"Programm den angegebenen Port, kann dieses Problem<br>" +
							"nur durch einen Neustart von ZUSMORO behoben werden!", "Fehler beim Verbindungsaufbau", JOptionPane.WARNING_MESSAGE);
			return;
		} catch (IOException e) {
			JOptionPane.showMessageDialog(MainFrame.mainFrame, "Die E/A-Streams für Port " + portName
					+ " konnten nicht geöffnet werden.", "Fehler beim Verbindungsaufbau", JOptionPane.WARNING_MESSAGE);
			return;
		} catch (TooManyListenersException e) {
			e.printStackTrace();
		} catch (UnsupportedCommOperationException e) {
			e.printStackTrace();
		}
//		try {
//			if (isConnected) {
//				writeToStream("v");
//			}
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
	}

	private void initSerialPort() throws TooManyListenersException,
			UnsupportedCommOperationException {
		serialPort.addEventListener(this);
		serialPort.notifyOnDataAvailable(true);
		serialPort
				.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
		//serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_RTSCTS_IN | SerialPort.FLOWCONTROL_RTSCTS_OUT);
		serialPort.setDTR(true);
		serialPort.setRTS(true);
		//serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
	}

	public void serialEvent(SerialPortEvent event) {
		//System.out.println("EVENT: " + event.getEventType());
		switch (event.getEventType()) {
		// case SerialPortEvent.BI:
		// case SerialPortEvent.OE:
		// case SerialPortEvent.FE:
		// case SerialPortEvent.PE:
		// case SerialPortEvent.CD:
		// case SerialPortEvent.CTS:
		// case SerialPortEvent.DSR:
		// case SerialPortEvent.RI:
		case SerialPortEvent.OUTPUT_BUFFER_EMPTY:
			break;
		case SerialPortEvent.DATA_AVAILABLE:
			//StringBuilder readBuffer = new StringBuilder(stringBuilderLength);
			System.out.println("DATA_AVAILABLE: " + serialCounter);
			int c;
			try {
				//System.out.println(in.read());
				//System.out.println("ddddd: " + d.readLine());
				String scannedInput = null;
	
				while( (scannedInput = d.readLine()) != null){//solange wir noch nicht alle readlines abgearbeitet haben müssen wir weiter machen
					
				//}
				//scannedInput = d.readLine();
					//if(scannedInput != null){
						if(!scannedInput.equals("")){
							System.out.println("scannedInput for " + serialCounter + ":" + scannedInput);
							setChanged();
							System.out.println("countObservers(): " + countObservers());
							notifyObservers(scannedInput);
						}else{
							setChanged();
							System.out.println("Void string..." + serialCounter);
							notifyObservers(scannedInput);
						}
					//}else{
					//	System.out.println("Nothing to read..." + serialCounter);
					//}
				}
				/*
				while ((c = inStream.read()) > -1) {
					if (c == 13 || c == 10) {
						System.out.println("c"+ serialCounter + ": " + (byte) c);
					}
					if (c != 13 && c != 10) {
						readBuffer.append((char) c);
						System.out.print((byte) c);
					} else if (c == 10) {
						String scannedInput = readBuffer.toString();
						byte[] ret = scannedInput.getBytes();
						System.out.print("scannedInput " + serialCounter + ": ");
						for(int j = 0;j < ret.length;j++){
							System.out.print(ret[j]);
						}
						System.out.println("");
						readBuffer = new StringBuilder(stringBuilderLength);
						setChanged();
						notifyObservers(scannedInput);
					}
				}*/
			} catch (IOException e) {
				// TODO hier etwas tun?
				System.out.println(e.getMessage());
				//setChanged();
				//notifyObservers("ZERO");
				
			}
			serialCounter++;
			break;
		}
	}

	public void writeToStream(final String msg) throws IOException {
		if (writeThread == null || !writeThread.isAlive()) {
			writeThread = new WriteThread();
			writeThread.start();
		}
		int queueSize = writeThread.writeMsg(msg);
		if (MainFrame.DEBUG) {
		System.out.println("Queue size: " + queueSize);
		}
		if (queueSize >= eva) {
			writeThread.stopThread();
			throw new IOException(
					"<html>Fehler bei der Übertragung der Befehle,<br>evtl. ist die EVA-Frequenz zu hoch eingestellt");
		}
	}

	public static ArrayList<String> getCOMPorts() {
		String portName;
		ArrayList<String> availablePorts = new ArrayList<String>();
		for (int i = 1; i < 51; i++) {
			try {
				portName = "COM" + i;
				CommPortIdentifier.getPortIdentifier(portName);
				availablePorts.add(portName);
			} catch (NoSuchPortException e) {
				// dann halt nicht
			}
		}
		return availablePorts;
	}

	public boolean isConnected() {
		return isConnected;
	}

	public void disconnect() {
		if (MainFrame.DEBUG) {
		System.out.println("Communicator disconnecting...");
		}
		if (writeThread != null) {
			if (MainFrame.DEBUG) {
			System.out.println("stopping WriterThread...");
			}
			writeThread.stopThread();
			if (MainFrame.DEBUG) {
			System.out.println("interrupting WriterThread...");
			}
			writeThread.interrupt();
		}
		writeThread = null;
		if (serialPort != null) {
			if (MainFrame.DEBUG) {
			System.out.println("closing Streams...");
			}
			try {
				inStream.close();
				outStream.close();
				inStream = null;
				outStream = null;
			} catch (IOException e) {
				e.printStackTrace();
				if (MainFrame.DEBUG) {
				System.out.println("Exception while closing Streams");
				}
			}
			if (MainFrame.DEBUG) {
			System.out.println("closing SerialPort...");
			}
			Thread closeThread = new Thread(new Runnable() {
				public void run() {
					SerialPort port = serialPort;
					serialPort = null;
					port.close();
				}
			});
			closeThread.setDaemon(true);
			closeThread.start();
			if (MainFrame.DEBUG) {
			System.out.println("SerialPort closed(?)...");
			}
		}
		isConnected = false;
		if (MainFrame.DEBUG) {
		System.out.println("Communicator disconnected!");
		}
	}

	private class WriteThread extends Thread {
		private ArrayList<String> messages;
		private volatile boolean stop;

		public WriteThread() {
			super("Communicator.WriteThread");
			messages = new ArrayList<String>(eva);//20
			stop = false;
			setDaemon(true);
		}

		public int writeMsg(String msg) {
			int size;
			synchronized (messages) {
				messages.add(msg);
				size = messages.size();
			}
			synchronized (writeSignal) {
				writeSignal.notify();
			}
			return size;
		}

		public void stopThread() {
			stop = true;
			if (MainFrame.DEBUG) {
				System.out.println("Stop-Flag set!");
			}
		}

		@Override
		public void run() {
			String msg;
			stop = false;
			while (!stop && !isInterrupted()) {
				msg = null;
				synchronized (messages) {
					if (!messages.isEmpty()) {
						msg = messages.get(0);
						messages.remove(0);
					}
				}
				if (msg != null) {
					try {
						if (MainFrame.DEBUG) {
							System.out.println("Writing to Stream: "+msg);
							System.out.print("Writing to Stream2: ");
							byte[] ret = msg.getBytes();
							for(int j = 0;j < ret.length;j++){
								System.out.print(ret[j]);
							}
							System.out.println("");
						}
						//byte[] bytes = msg.getBytes();
						//outStream.write((msg + (msg.endsWith("\r\n") ? "" : "\r\n")).getBytes());
						outStream.write(msg.getBytes());
					} catch (IOException e) {
						Automat.runningAutomat.requestCancel("<html>Übermittlung zum Roboter fehlgeschlagen:<br>"
								+ e.getMessage() + "</html>", true);
						break;
					}
				} else {
					try {
						synchronized (writeSignal) {
							writeSignal.wait(1000);
						}
					} catch (InterruptedException e) {
						if (MainFrame.DEBUG) {
							System.out.println("WriteThread interrupted!");
						}
						break;
					}
				}
			}
			if (MainFrame.DEBUG) {
				System.out.println("WriteThread died!");
			}
		}
	}
}
