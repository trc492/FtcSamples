/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.LinkedList;
import java.util.Locale;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

/**
 * This class implements a Homography Mapper. It can be used to map the vision camera's pixel coordinates to the
 * robot's real world coordinates.
 */
public class TrcHomographyMapper
{
    /**
     * This class implements a homography rectangle with the four coordinate points. The rectangle can be a real
     * world rectangle or a vision camera's pixel rectangle.
     */
    public static class Rectangle
    {
        public Point topLeft;
        public Point topRight;
        public Point bottomLeft;
        public Point bottomRight;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param topLeft specifies the top left point of the rectangle.
         * @param topRight specifies the top right point of the rectangle.
         * @param bottomLeft specifies the bottom left point of the rectangle.
         * @param bottomRight specifies the bottom right point of the rectangle.
         */
        public Rectangle(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight)
        {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
        }   //Rectangle

        /**
         * Constructor: Create an instance of the object.
         *
         * @param topLeftX specifies the top left X of the rectangle.
         * @param topLeftY specifies the top left Y of the rectangle.
         * @param topRightX specifies the top right X of the rectangle.
         * @param topRightY specifies the top right Y of the rectangle.
         * @param bottomLeftX specifies the bottom left X of the rectangle.
         * @param bottomLeftY specifies the bottom left Y of the rectangle.
         * @param bottomRightX specifies the bottom right X of the rectangle.
         * @param bottomRightY specifies the bottom right Y of the rectangle.
         */
        public Rectangle(double topLeftX, double topLeftY, double topRightX, double topRightY,
                         double bottomLeftX, double bottomLeftY, double bottomRightX, double bottomRightY)
        {
            this(new Point(topLeftX, topLeftY), new Point(topRightX, topRightY),
                 new Point(bottomLeftX, bottomLeftY), new Point(bottomRightX, bottomRightY));
        }   //Rectangle

        public String toString()
        {
            return String.format(Locale.US, "[topLeft%s, topRight%s, bottomLeft%s, bottomRight%s]",
                    topLeft.toString(), topRight.toString(), bottomLeft.toString(), bottomRight.toString());
        }   //toString

    }   //class Rectangle

    private final Mat homographyMatrix;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param srcTopLeft specifies the source top left point (e.g. top left of camera pixel coordinate).
     * @param srcTopRight specifies the source top right point (e.g. top right of camera pixel coordinate).
     * @param srcBottomLeft specifies the source bottom left point (e.g. bottom left of camera pixel coordinate).
     * @param srcBottomRight specifies the source bottom right point (e.g. bottom right of camera pixel coordinate).
     * @param dstTopLeft specifies the destination top left point (e.g. top left of robot world coordinate).
     * @param dstTopRight specifies the destination top right point (e.g. top right of robot world coordinate).
     * @param dstBottomLeft specifies the destination bottom left point (e.g. bottom left of robot world coordinate).
     * @param dstBottomRight specifies the destination bottom right point (e.g. bottom right of robot world coordinate).
     */
    public TrcHomographyMapper(
        Point srcTopLeft, Point srcTopRight, Point srcBottomLeft, Point srcBottomRight,
        Point dstTopLeft, Point dstTopRight, Point dstBottomLeft, Point dstBottomRight)
    {
        LinkedList<Point> srcList = new LinkedList<>();
        LinkedList<Point> dstList = new LinkedList<>();
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        MatOfPoint2f dstPoints = new MatOfPoint2f();

        srcList.add(srcTopLeft);
        dstList.add(dstTopLeft);
        srcList.add(srcTopRight);
        dstList.add(dstTopRight);
        srcList.add(srcBottomLeft);
        dstList.add(dstBottomLeft);
        srcList.add(srcBottomRight);
        dstList.add(dstBottomRight);

        srcPoints.fromList(srcList);
        dstPoints.fromList(dstList);

        // Find the 3x3 homography matrix.
        homographyMatrix = Calib3d.findHomography(srcPoints, dstPoints);
        // release MatOfPoint2f to prevent memory leak.
        srcPoints.release();
        dstPoints.release();
    }   //TrcHomographyMapper

    /**
     * Constructor: Create an instance of the object.
     *
     * @param srcRect specifies the source rectangle (e.g. camera pixel coordinate).
     * @param dstRect specifies the destination rectangle (e.g. robot world coordinate).
     */
    public TrcHomographyMapper(Rectangle srcRect, Rectangle dstRect)
    {
        this(srcRect.topLeft, srcRect.topRight, srcRect.bottomLeft, srcRect.bottomRight,
             dstRect.topLeft, dstRect.topRight, dstRect.bottomLeft, dstRect.bottomRight);
    }   //TrcHomographyMapper

    /**
     * This method maps a source point to the destination point using the homography matrix.
     *
     * @param srcPoint specifies the source point.
     * @return the mapped destination point.
     */
    public Point mapPoint(Point srcPoint)
    {
        double[][] pmat = new double[3][1];
        pmat[0][0] = srcPoint.x;
        pmat[1][0] = srcPoint.y;
        pmat[2][0] = 1.0;

        double[][] h = new double[3][3];

        for (int i = 0; i < homographyMatrix.rows(); i++)
        {
            for (int j = 0; j < homographyMatrix.cols(); j++)
            {
                h[i][j] = homographyMatrix.get(i, j)[0];
            }
        }

        double[][] result = multiply(h, pmat);

        // Results need to be scaled by the Z-axis.
        for (int i = 0; i < result.length; i++)
        {
            result[i][0] *= (1.0 / result[2][0]);
        }

        return new Point(result[0][0], result[1][0]);
    }   //mapPoint

    /**
     * This method multiplies two matrixes. The multiply in Core.gemm is not functioning correctly for the job.
     *
     * @param firstMatrix specifies the first matrix for the multiplication.
     * @param secondMatrix specifies the second matrix for the multiplication.
     * @return product matrix.
     */
    private double[][] multiply(double[][] firstMatrix, double[][] secondMatrix)
    {
        double[][] product = new double[firstMatrix.length][secondMatrix[0].length];

        for (int i = 0; i < firstMatrix.length; i++)
        {
            for (int j = 0; j < secondMatrix[0].length; j++)
            {
                for (int k = 0; k < firstMatrix[0].length; k++)
                {
                    product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }

        return product;
    }   //multiply

}   //class TrcHomographyMapper
