package nbtool.images;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;

import nbtool.util.Logger;

public class RobotImage extends ImageParent {
	public byte whiteData[];

	public RobotImage(int w, int h, byte[] d) {
		super(w, h, d);
	}

	@Override
	public int pixelSize() {
		return -1;
	}

	private static final Color[] angleMap = initColorMap();

	private static Color[] initColorMap(){

		Color[] ret = new Color[256];
		Color[] top = {Color.RED,
				Color.ORANGE,
				Color.YELLOW,
				Color.GREEN,
				Color.BLUE,
				Color.MAGENTA,
				Color.GRAY,
				Color.PINK};
		assert(top.length == 8);

		for (int i = 0; i < 256; ++i) {
			int bi = i / 32;
			int ni = (bi + 1) % 8;

			int dist = i % 32;

			Color bc = top[bi];
			Color nc = top[ni];

			int dr = nc.getRed() - bc.getRed();
			int dg = nc.getGreen() - bc.getGreen();
			int db = nc.getBlue() - bc.getBlue();

			Color tc = new Color(
					bc.getRed() + (dist * dr) / 32,
					bc.getGreen() + (dist * dg) / 32,
					bc.getBlue() + (dist * db) / 32
					);
			ret[i] = tc;
		}

		return ret;
	};

	public void setWhiteData(byte[] whitedata) {
		whiteData = whitedata;
	}

	@Override
	public BufferedImage toBufferedImage() {

		BufferedImage ret = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);

		for (int x = 0; x < width; ++x) {
			for (int y = 0; y < height; ++y) {
				ret.setRGB(x, y, Color.BLACK.getRGB());
			}
		}

		int bottom[] = new int[width];              // not really used
		int top[] = new int[width];
		int maxbottom[] = new int[width];           // lowest edge that points up
		int mintop[] = new int[width];              // highest edge that points down
		boolean evidence[] = new boolean[width];    // is there evidence of an obstacle in this column?
		// initialize accumulators
		for (int i = 0; i < width; i++) {
			bottom[i] = height;
			top[i] = 0;
			maxbottom[i] = 0;
			mintop[i] = height;
			evidence[i] = false;
		}

		try {
			DataInputStream dis = new DataInputStream(new ByteArrayInputStream(data));
			int n_edges = data.length / (4 * 4);
			Logger.logf(Logger.INFO, "%d edges expected.", n_edges);

			// loop through all of the edges
			for (int i = 0; i < n_edges; ++i) {
				int x = dis.readInt();
				int y = dis.readInt();
				int mag = dis.readInt();  // magnitude - could be useful?
				int ang = dis.readInt();  // where the edge points

				Color base = angleMap[ang];

				// for every edge look at where it is and where it points
				// NOTE: in Vision, edges are also marked by whether or not they
				//       are in Hough lines. These edges can and should be ignored.
				//       There is a "memberOf" method to check this.
				if (y < height - 15) {           // hack to ignore own chest
					// is edge pointing up or down?
					if (ang < 128) {
						if (y < bottom[x]) {
							bottom[x] = y;
						}
						if (y > maxbottom[x]) {
							maxbottom[x] = y;
						}
					} else {
						if (y > top[x]) {
							top[x] = y;
						}
						if (y < mintop[x]) {
							mintop[x] = y;
						}
					}
				}
			}
		} catch (Exception e) {
			Logger.logf(Logger.ERROR, "Conversion from bytes to EdgeImage to BufferedImage failed.");
			e.printStackTrace();
			return null;
		}

		int minLength = 35;    // needs to be a bit more scientific, e.g. based on field cross size
		                       // also needs to be relative to image width
		for (int i = 0; i < width; i++) {
			// if there is an upward pointing edge in this column
			if (maxbottom[i] != 0) {
				int start = i;        // start of the run of columns
				int accum = 0;        // silly attempt to find how much variation in the run
				int whiteCaps = 0;    // how many corresponding downward edges are there in the run
				int whiteTop = 0;     // how many columns are mostly white
				i++;                  // step through the columns
				// look for good columns, or columns surrounded by other good columns
				while (i < width - 2 &&
					   (maxbottom[i] != 0 || maxbottom[i+1] != 0) || mintop[i] < height * 3 / 4) {
					if (maxbottom[i] == 0) {
						maxbottom[i] = maxbottom[i - 1];
					}
					accum += Math.abs(maxbottom[i] - maxbottom[i-1]);
					i++;
					if (mintop[i] < maxbottom[i]) {
						whiteCaps++;
					}
					// this is the key business, look for mostly white between edge and top
					int w = 0;
					for (int r = 0; r < maxbottom[i]; r++) { // loop from top to edge
						if (((whiteData[r * width + i]) & 0xFF) > 128) {
							w++;
						}
					}
					if (w > maxbottom[i] / 2) { // more than half of points are "white"
						whiteTop++;
					}
					evidence[i] = true;
				}
				int span = i - start;
				accum = accum / span;
				// if wide enough, varied enough and white enough
				if (span > minLength && accum > 0 && whiteCaps < span * 2 / 3 &&
					(whiteTop > span / 3 || whiteTop > minLength)) {
					System.out.print("Found robot chunk of length "+(i - start)+ " from ");
					System.out.println(start+" to "+i);
					System.out.println("Accum average = "+accum);
					System.out.println("White caps = "+whiteCaps);
					System.out.println("White tops = "+whiteTop);
				} else {
					System.out.println("Surpressing noise at "+start+" "+i);
					if (span > minLength) {
						System.out.println("Accum average = "+accum);
						System.out.println("White caps = "+whiteCaps);
						System.out.println("White tops = "+whiteTop);
					}
					for (int j = start; j < i; j++) {
						//maxbottom[j] = 0;
						evidence[j] = false;
					}
				}
			}
		}

		// this bit is just to draw the edges that we found
		for (int i = 0; i < width; i++) {
			if (maxbottom[i]== 0) {
				// nothing to see here
			} else {
				// I get paranoid around the edges of the image
				if (bottom[i] > height -1) {
					bottom[i] = height - 1;
				}
				// not really relevant, but it makes things look a bit nicer
				// here we're just printing the downward edges
				if (top[i] != 0) {
					if (maxbottom[i] > mintop[i] && evidence[i]) {
						ret.setRGB(i, mintop[i], angleMap[64].getRGB());
					}
				}
				// if we have evidence paint the edge red, otherwise paint it blue
				if (evidence[i]) {
					ret.setRGB(i, maxbottom[i], angleMap[0].getRGB());
				} else {
					ret.setRGB(i, maxbottom[i], angleMap[128].getRGB());
				}
			}
		}

		return ret;
	}

	@Override
	public String encoding() {
		return "n/a";
	}

}
