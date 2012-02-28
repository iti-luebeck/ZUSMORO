package robots.epuck;

import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Observable;
import java.util.TooManyListenersException;

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

	public Communicator(String portName) {
		if (MainFrame.DEBUG) {
		System.out.println("New Communicator for: "+portName);
		}
		try {
			portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
			serialPort = (SerialPort) portIdentifier.open("Communicator", 2000);
			inStream = serialPort.getInputStream();
			outStream = serialPort.getOutputStream();
			serialPort.addEventListener(this);
			serialPort.notifyOnDataAvailable(true);
			serialPort
					.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
			serialPort.setDTR(true);
			serialPort.setRTS(true);
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

	public void serialEvent(SerialPortEvent event) {
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
			StringBuilder readBuffer = new StringBuilder();
			//System.out.println("DATA_AVAILABLE");
			int c;
			try {
				while ((c = inStream.read()) > -1) {
					if (c != 13 && c != 10) {
						readBuffer.append((char) c);
						//System.out.print((char) c);
					} else if (c == 10) {
						String scannedInput = readBuffer.toString();
						readBuffer = new StringBuilder();
						setChanged();
						notifyObservers(scannedInput);
					}
				}
			} catch (IOException e) {
				// TODO hier etwas tun?
			}
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
		if (queueSize >= 20) {
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
			messages = new ArrayList<String>(20);
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
