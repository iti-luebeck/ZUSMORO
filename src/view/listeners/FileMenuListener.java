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
