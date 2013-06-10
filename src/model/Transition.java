package model;

import java.awt.Graphics;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Path2D;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JPanel;

import model.ChangeEvent.ChangeEventType;
import model.bool.BooleanExpression;
import smachGenerator.ISmachableGuard;
import smachGenerator.ISmachableTransition;
import view.MainFrame;
import view.TransitionView;
import view.EditorPanel.EditorMode;

public class Transition extends JButton implements MouseListener,
		MouseMotionListener, KeyListener, ISmachableTransition {

	public static int createdTransitions = 0;

	private static final long serialVersionUID = -5943400074027452139L;
	private State rootState;
	private Point rootStateLocation;
	private State followerState;
	private Point followerStateLocation;
	private String label;
	private BooleanExpression guard;
	private Path2D path;
	private ArrayList<Point> pathPoints;
	private boolean mouseOnMe = false;

	private int startX = 0;
	private int xOffset = 0;
	private int stringX;
	private int stringY;
	private boolean rebuildPath = true;

	public Transition(State root, State follower) {
		super();
		createdTransitions++;
		setLabel("T" + createdTransitions);
		this.rootState = root;
		this.guard = BooleanExpression.TRUE;
		this.followerState = follower;
		this.xOffset = (int) ((root.getTransitions().size()) * -30 * Math.pow(
				-1.0, root.getTransitions().size()));
		this.buildPath(0);
		this.addMouseListener(this);
		this.addMouseMotionListener(this);
		addKeyListener(this);
		setToolTipText("<html><b>" + label + "</b><br>true</html>");
	}

	Transition(State root, State follower, String label, int offset,
			BooleanExpression guard) {
		super();
		Transition.createdTransitions++;
		this.label = label;
		this.xOffset = offset;
		this.rootState = root;
		this.guard = guard;
		this.followerState = follower;
		this.buildPath(0);
		this.addMouseListener(this);
		this.addMouseMotionListener(this);
		addKeyListener(this);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see model.ISmachableTransition#getFollowerState()
	 */
	@Override
	public State getFollowerState() {
		return this.followerState;
	}

	public State getRootState() {
		return this.rootState;
	}

	public BooleanExpression getGuard() {
		return this.guard;
	}
	
	@Override
	public ISmachableGuard getSmachableGuard() {
		if (guard instanceof ISmachableGuard) {
			return (ISmachableGuard) guard;
		} else {
			return null;
		}
	}

	public void setGuard(BooleanExpression guard) {
		this.guard = guard;
	}

	public boolean eval() {
		return guard.eval();
	}

	public String getLabel() {
		return this.label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	public Path2D buildPath(int offset) {
		// System.out.println(offset);
		Point point = rootState.getLocation();
		point.x = point.x + State.STATE_WIDTH / 2;
		point.y = point.y + State.STATE_HEIGHT;
		Point fpoint = followerState.getLocation();
		fpoint.x = fpoint.x + State.STATE_WIDTH / 2;
		if (!point.equals(rootStateLocation)
				|| !fpoint.equals(followerStateLocation) || rebuildPath) {
			rebuildPath = false;
			rootStateLocation = point;
			followerStateLocation = fpoint;
			int deltaY = fpoint.y - point.y;
			int deltaX = fpoint.x - point.x;
			path = new Path2D.Double();
			path.moveTo(point.x, point.y);
			if (deltaY >= 0) {
				if (xOffset == 0) {
					bezierPath(point, fpoint);
				} else {
					straightPath(point, fpoint);
				}
			} else {
				if (xOffset == 0) {
					bottomUpPath(point, fpoint);
				} else {
					straightPath(point, fpoint);
				}
			}
			stringX = point.x + deltaX / 2 + 10 + xOffset;
			stringY = point.y + deltaY / 2;
			path.moveTo(stringX - 20, stringY);
			path.lineTo(stringX, stringY);
			arrowtips(fpoint);
			Rectangle bounds = path.getBounds();
			bounds = new Rectangle(bounds.x - 2, bounds.y - 2,
					bounds.width + 4, bounds.height + 4);
			// System.out.println(bounds);
			setBounds(bounds);
		}
		return path;
	}

	private void arrowtips(Point fpoint) {
		double tipX = fpoint.x - 7;
		double tipY = fpoint.y - 7;
		path.moveTo(tipX, tipY);
		path.lineTo(fpoint.x, fpoint.y);
		tipX = fpoint.x + 7;
		path.moveTo(tipX, tipY);
		path.lineTo(fpoint.x, fpoint.y);
	}

	private void bottomUpPath(Point point, Point fpoint) {
		int deltaY = fpoint.y - point.y;
		int deltaX = fpoint.x - point.x;
		int minY = Math.min(-deltaY, 30);
		int maxX = 50;
		if (deltaX >= 0) {
			Point nextPoint = new Point(point.x + maxX, point.y);
			Point nextfPoint = new Point(fpoint.x - maxX, fpoint.y);
			path.curveTo(point.x, point.y + minY, nextPoint.x, point.y + minY,
					nextPoint.x, nextPoint.y);
			bezierPath(nextPoint, nextfPoint);
			// straightDownPath(nextPoint, nextfPoint);
			path.curveTo(nextfPoint.x, nextfPoint.y - minY, fpoint.x, fpoint.y
					- minY, fpoint.x, fpoint.y);
		} else {
			Point nextPoint = new Point(point.x - maxX, point.y);
			Point nextfPoint = new Point(fpoint.x + maxX, fpoint.y);
			path.curveTo(point.x, point.y + minY, nextPoint.x, point.y + minY,
					nextPoint.x, nextPoint.y);
			bezierPath(nextPoint, nextfPoint);
			// straightDownPath(nextPoint, nextfPoint);
			path.curveTo(nextfPoint.x, nextfPoint.y - minY, fpoint.x, fpoint.y
					- minY, fpoint.x, fpoint.y);
		}
	}

	private void straightPath(Point point, Point fpoint) {
		pathPoints = new ArrayList<Point>();
		// int deltaY = fpoint.y - point.y;
		int deltaX = fpoint.x - point.x;
		pathPoints.add(new Point(point));

		path.lineTo(point.x, point.y + 10);
		pathPoints.add(new Point(point.x, point.y + 10));

		path.lineTo(point.x + deltaX / 2 + xOffset, point.y + 10);
		pathPoints.add(new Point(point.x + deltaX / 2 + xOffset, point.y + 10));

		path.lineTo(point.x + deltaX / 2 + xOffset, fpoint.y - 10);
		pathPoints
				.add(new Point(point.x + deltaX / 2 + xOffset, fpoint.y - 10));

		path.lineTo(fpoint.x, fpoint.y - 10);
		pathPoints.add(new Point(fpoint.x, fpoint.y - 10));

		path.lineTo(fpoint.x, fpoint.y);
		pathPoints.add(new Point(fpoint));
	}

	private void bezierPath(Point point, Point fpoint) {
		int deltaY = fpoint.y - point.y;
		int deltaX = fpoint.x - point.x;
		path.curveTo(point.x + xOffset, point.y + deltaY / 2, point.x + deltaX
				+ xOffset, point.y + deltaY / 2, fpoint.x, fpoint.y);

	}

	public boolean getMouseOn() {
		return this.mouseOnMe;
	}

	public int getXOffset() {
		return xOffset;
	}

	public int getStringX() {
		return stringX;
	}

	public int getStringY() {
		return stringY;
	}

	@Override
	public boolean contains(int x, int y) {
		int absX = x + getX();
		int absY = y + getY();
		// System.out.println("contains ("+x+","+y+")");
		boolean contained = path.intersects(absX - 2, absY - 2, 4, 4);
		contained = contained && (xOffset == 0 || pathContains(absX, absY));
		// if (contained && !mouseOnMe) {
		// mouseOnMe = true;
		// ((JPanel) getParent()).repaint(getBounds());
		// } else if (!contained && mouseOnMe) {
		// mouseOnMe = false;
		// ((JPanel) getParent()).repaint(getBounds());
		// }
		return contained;
	}

	@Override
	public boolean contains(Point p) {
		return contains(p.x, p.y);
	}

	private boolean pathContains(int x, int y) {
		double distance = 42.0;
		if (pathPoints != null) {
			Point lineStart = pathPoints.get(0);
			Point lineEnd;
			for (int i = 1; i < pathPoints.size(); i++) {
				lineEnd = pathPoints.get(i);
				distance = Math.min(
						distance,
						lineDistance(lineStart.x, lineStart.y, lineEnd.x,
								lineEnd.y, x, y));
				lineStart = lineEnd;
			}
		}
		return distance <= 5.0;
	}

	private double lineDistance(int lx1, int ly1, int lx2, int ly2, int px,
			int py) {
		double a1 = lx2 - lx1;
		double a2 = ly2 - ly1;
		double b1 = px - lx1;
		double b2 = py - ly1;
		double alpha = (a1 * b1 + a2 * b2) / (a1 * a1 + a2 * a2);
		double result;
		if (alpha > 1.0 || alpha < 0.0) {
			result = Double.MAX_VALUE;
		} else {
			a1 = alpha * a1;
			a2 = alpha * a2;
			result = Math.sqrt(Math.pow((a1 - b1), 2.0)
					+ Math.pow((a2 - b2), 2.0));
		}
		return result != result ? Double.MAX_VALUE : result;
	}

	@Override
	protected void paintComponent(Graphics g) {
		// paint nothing here
	}

	public void mouseClicked(MouseEvent e) {
		// System.out.println("event" + e);
		if (e.getClickCount() == 2 && Automat.runningAutomat == null) {
			// if (MainFrame.programm.getRobot() != null) {
			// MainFrame.programm.getRobot().getTransitionView(this).setVisible(true);
			// } else {
			// try {
			// MainFrame.robotClass.newInstance().getTransitionView(this).setVisible(true);
			// } catch (InstantiationException e1) {
			// // Auto-generated catch block
			// e1.printStackTrace();
			// } catch (IllegalAccessException e1) {
			// // Auto-generated catch block
			// e1.printStackTrace();
			// }
			// }
			new TransitionView(this).setVisible(true);
		} else if (e.getClickCount() == 1 && Automat.runningAutomat == null
				&& MainFrame.editorPanel.getMode() == EditorMode.DELETE) {
			rootState.removeTransition(this);
			MainFrame.automat.setChanged(new ChangeEvent(
					ChangeEventType.TRANSITION_DELETE, this, false));
		}
	}

	public void mouseEntered(MouseEvent e) {
		mouseOnMe = true;
		((JPanel) getParent()).repaint(getBounds());
		requestFocusInWindow();
		// System.out.println("event"+e);
	}

	public void mouseExited(MouseEvent e) {
		mouseOnMe = false;
		((JPanel) getParent()).repaint(getBounds());
		getParent().requestFocusInWindow();
		// System.out.println("event"+e);
	}

	public void mousePressed(MouseEvent e) {
		// System.out.println("event"+e);
		startX = e.getXOnScreen();
	}

	public void mouseReleased(MouseEvent e) {
		// xOffset = e.getX() - startX;
	}

	public void mouseDragged(MouseEvent e) {
		// System.out.println("xOffset = " + xOffset);
		xOffset += e.getXOnScreen() - startX;
		startX = e.getXOnScreen();
		rebuildPath = true;
		this.buildPath(0);
		// Rectangle bounds = getBounds();
		((JPanel) this.getParent()).repaint();
	}

	public void mouseMoved(MouseEvent arg0) {
		// nothing
	}

	public void keyPressed(KeyEvent e) {
		// nothing
	}

	public void keyReleased(KeyEvent e) {
		if (e.getKeyCode() == KeyEvent.VK_DELETE
				&& Automat.runningAutomat == null) {
			rootState.removeTransition(this);
			MainFrame.automat.setChanged(new ChangeEvent(
					ChangeEventType.TRANSITION_DELETE, this, false));
		}
	}

	public void keyTyped(KeyEvent e) {
		// nothing
	}
}
