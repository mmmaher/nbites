package nbtool.gui.logviews.images;

import java.awt.Graphics;
import java.awt.Color;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.util.Vector;

import nbtool.util.Logger;
import nbtool.data.Log;
import nbtool.gui.logviews.misc.ViewParent;
import nbtool.images.EdgeImage;
import nbtool.images.RobotImage;
import nbtool.images.Y8image;
import nbtool.io.CommonIO.IOFirstResponder;
import nbtool.io.CommonIO.IOInstance;
import nbtool.io.CrossIO;
import nbtool.io.CrossIO.CrossFunc;
import nbtool.io.CrossIO.CrossInstance;
import nbtool.io.CrossIO.CrossCall;
import nbtool.util.Utility;

public class RobotView extends ViewParent implements IOFirstResponder {

    int width;
    int height;

    int displayw;
    int displayh;

    final int fieldw = 640;
    final int fieldh = 554;

    final int buffer = 5;

    double resize = 1;

    boolean click = false;
    boolean drag = false;

    // Click and release values
    int clickX1 = 0;
    int clickY1 = 0;
    int clickX2 = 0;
    int clickY2 = 0;

    // Field coordinate image upper left hand corder
    int fx0;
    int fy0;

    // Center of field cordinate system
    int fxc = displayw + buffer + fieldw/2;
    int fyc = fieldh;

    BufferedImage originalImage;
    BufferedImage edgeImage;
	BufferedImage whiteImage;
	BufferedImage edgeImage2;
    Vector<Double> lines;


    @Override
    public void setLog(Log newlog) {
        CrossInstance ci = CrossIO.instanceByIndex(0);
        if (ci == null)
            return;
        CrossFunc func = ci.functionWithName("Vision");
        assert(func != null);

        CrossCall cc = new CrossCall(this, func, newlog);

        assert(ci.tryAddCall(cc));

        // TODO: Don't hard code SExpr paths
        width =  newlog.tree().get(4).get(1).get(5).get(1).valueAsInt() / 2;
        height = newlog.tree().get(4).get(1).get(6).get(1).valueAsInt() / 2;

        displayw = width*2;
        displayh = height*2;

        fx0 = displayw + buffer;
        fy0 = 0;

        fxc = displayw + buffer + fieldw/2;
        fyc = fieldh;

        originalImage = Utility.biFromLog(newlog);
    }

    public void paintComponent(Graphics g) {
        if (edgeImage != null) {
            g.drawImage(originalImage, 0, 0, displayw, displayh, null);
            g.drawImage(edgeImage, 0, displayh + buffer, displayw, displayh, null);
			g.drawImage(whiteImage, displayw+10, 0, null);
			g.drawImage(edgeImage2, displayw+10, displayh, null);

        }
    }

    public RobotView() {
        super();
        setLayout(null);
        lines = new Vector<Double>();

        this.addMouseListener(new DistanceGetter());
    }

    class DistanceGetter implements MouseListener {

      public void mouseClicked(MouseEvent e) {
        clickX1 = e.getX();
        clickY1 = e.getY();
        click = true;
        repaint();
      }

      public void mousePressed(MouseEvent e) {
        clickX1 = e.getX();
        clickY1 = e.getY();
      }

      public void mouseReleased(MouseEvent e) {
        clickX2 = e.getX();
        clickY2 = e.getY();
        if (clickX1 != clickX2 || clickY1 != clickY2) {
            drag = true;
            repaint();
        }
      }

      public void mouseEntered(MouseEvent e) {}

      public void mouseExited(MouseEvent e) {}
    }

    @Override
    public void ioFinished(IOInstance instance) {}

    @Override
    public void ioReceived(IOInstance inst, int ret, Log... out) {
		Y8image white8 = new Y8image(width, height, out[1].bytes);
		this.whiteImage = white8.toBufferedImage();

        EdgeImage ei1 = new EdgeImage(width, height,  out[5].bytes);
        edgeImage2 = ei1.toBufferedImage();


        RobotImage ei = new RobotImage(width, height,  out[5].bytes);
		ei.setWhiteData(out[1].bytes);
        edgeImage = ei.toBufferedImage();
        repaint();

        // TODO refactor. Protobuff?
        byte[] lineBytes = out[6].bytes;
        int numLines = lineBytes.length / (18 * 4);
        Logger.logf(Logger.INFO, "%d field lines expected.", numLines);
        try {
            DataInputStream dis = new DataInputStream(new ByteArrayInputStream(lineBytes));
            for (int i = 0; i < numLines; ++i) {
                lines.add(dis.readDouble()); // image coord r
                lines.add(dis.readDouble()); // image coord t
                lines.add(dis.readDouble()); // image coord ep0
                lines.add(dis.readDouble()); // image coord ep1
                lines.add((double)dis.readInt()); // hough index
                lines.add((double)dis.readInt()); // fieldline index
                lines.add(dis.readDouble()); // field coord r
                lines.add(dis.readDouble()); // field coord t
                lines.add(dis.readDouble()); // field coord ep0
                lines.add(dis.readDouble()); // field coord ep1
            }
        } catch (Exception e) {
            Logger.logf(Logger.ERROR, "Conversion from bytes to hough coord lines in LineView failed.");
            e.printStackTrace();
        }
    }

	@Override
	public boolean ioMayRespondOnCenterThread(IOInstance inst) {
		return false;
	}
}
