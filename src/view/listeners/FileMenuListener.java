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

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileNameExtensionFilter;

import model.Automat;
import model.Transition;
import view.EditorPanel;
import view.MainFrame;

public class FileMenuListener implements ActionListener {

	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("newFile")) {
			if (MainFrame.automat.getStateCount() > 0) {
				int confirm = JOptionPane.showConfirmDialog((Component) e.getSource(),
						"<html>Soll das bestehende Programm<br>wirklich verworfen werden?</html>",
						"Neues Programm anlegen", JOptionPane.YES_NO_OPTION);
				if (confirm == JOptionPane.YES_OPTION) {
					MainFrame.mainFrame.setEditorPanel(new EditorPanel(MainFrame.automat.getRobot()));
					Transition.createdTransitions = 0;
				}
			} else {
				// Wenn Programm leer brauch man kein neues anlegen
				MainFrame.editorPanel.resetUndoRedoQueue();
				Transition.createdTransitions = 0;
			}
		} else if (e.getActionCommand().equals("openFile")) {
			if (MainFrame.automat.getStateCount() > 0) {
				int confirm = JOptionPane.showConfirmDialog((Component) e.getSource(),
						"<html>Soll das bestehende Programm<br>wirklich verworfen werden?</html>", "Datei öffnen",
						JOptionPane.YES_NO_OPTION);
				if (confirm == JOptionPane.YES_OPTION) {
					JFileChooser fChooser = new JFileChooser();
					fChooser.setFileFilter(new FileNameExtensionFilter("ZUSMORO xml-Datei (*.xml)", "xml"));
					int choosen = fChooser.showOpenDialog((Component) e.getSource());
					if (choosen == JFileChooser.APPROVE_OPTION) {
						try {
							Transition.createdTransitions = 0;
							MainFrame.automat = Automat.loadAutomat(fChooser.getSelectedFile());
						} catch (Exception e1) {
							JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Fehler beim Laden des Programms<br>"
									+ e1.getMessage() + "</html>", "Laden gescheitert", JOptionPane.WARNING_MESSAGE);
						}
						MainFrame.editorPanel.validateProgramm();
						MainFrame.automat.addObserver(MainFrame.editorPanel);
					}
				}
			} else {
				JFileChooser fChooser = new JFileChooser();
				fChooser.setFileFilter(new FileNameExtensionFilter("ZUSMORO xml-Datei (*.xml)", "xml"));
				int choosen = fChooser.showOpenDialog((Component) e.getSource());
				if (choosen == JFileChooser.APPROVE_OPTION) {
					try {
						Transition.createdTransitions = 0;
						MainFrame.automat = Automat.loadAutomat(fChooser.getSelectedFile());
					} catch (Exception e1) {
						JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Fehler beim Laden des Programms<br>"
								+ e1.getMessage() + "</html>", "Laden gescheitert", JOptionPane.WARNING_MESSAGE);
					}
					MainFrame.editorPanel.validateProgramm();
					MainFrame.automat.addObserver(MainFrame.editorPanel);
				}
			}
		} else if (e.getActionCommand().equals("saveFile")) {
			String fileName = MainFrame.automat.getSaveToFile();
			if (fileName != null) {
				try {
					MainFrame.automat.saveAutomat(new File(fileName));
				} catch (IOException e1) {
					JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Fehler beim Speichern des Programms<br>"
							+ e1.getMessage() + "</html>", "Speichern gescheitert", JOptionPane.WARNING_MESSAGE);
				}
			} else {
				JFileChooser fChooser = new JFileChooser();
				fChooser.setFileFilter(new FileNameExtensionFilter("ZUSMORO xml-Datei (*.xml)", "xml"));
				int choosen = fChooser.showSaveDialog((Component) e.getSource());
				if (choosen == JFileChooser.APPROVE_OPTION) {
					try {
						MainFrame.automat.saveAutomat(fChooser.getSelectedFile());
					} catch (IOException e1) {
						JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Fehler beim Speichern des Programms<br>"
								+ e1.getMessage() + "</html>", "Speichern gescheitert", JOptionPane.WARNING_MESSAGE);
					}
				}
			}
		} else if (e.getActionCommand().equals("saveUnder")) {
			JFileChooser fChooser = new JFileChooser();
			fChooser.setFileFilter(new FileNameExtensionFilter("ZUSMORO xml-Datei (*.xml)", "xml"));
			int choosen = fChooser.showSaveDialog((Component) e.getSource());
			if (choosen == JFileChooser.APPROVE_OPTION) {
				try {
					MainFrame.automat.saveAutomat(fChooser.getSelectedFile());
				} catch (IOException e1) {
					JOptionPane.showMessageDialog(MainFrame.mainFrame, "<html>Fehler beim Speichern des Programms<br>"
							+ e1.getMessage() + "</html>", "Speichern gescheitert", JOptionPane.WARNING_MESSAGE);
				}
			}
		} else if (e.getActionCommand().equals("print")) {
			// TODO Programm drucken
		} else if (e.getActionCommand().equals("exit")) {
			if (MainFrame.automat.getStateCount() > 0) {
				int confirm = JOptionPane.showConfirmDialog((Component) e.getSource(),
						"<html>Soll das bestehende Programm<br>wirklich verworfen werden?</html>",
						"Programm schließen", JOptionPane.YES_NO_OPTION);
				if (confirm == JOptionPane.YES_OPTION) {
					System.exit(0);
				}
			} else {
				System.exit(0);
			}
		}
	}
}
