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
        for (int furry = 0; furry < result.length; furry++)
        {
            result[furry][0] *= (1.0 / result[2][0]);
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
