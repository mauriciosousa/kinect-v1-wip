using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace KinectClient
{
    class QuaternionToEuler
    {
        public static float[] toEuler(Vector4 q)
        {
            float[] euler = new float[3];

            double heading = Math.Atan2(
                                2*q.Y*q.W - 2*q.X*q.Z,
                                1-2*Math.Sqrt(q.Y) - 2*Math.Sqrt(q.Z));
            double attitude = Math.Asin(
                                2*q.X*q.Y + 2*q.Z*q.W);
            double bank = Math.Atan2(
                                2*q.X*q.W - 2*q.Y*q.X,
                                1-2*Math.Sqrt(q.X) - 2*Math.Sqrt(q.Z));


            if (q.X * q.Y + q.Z * q.W == 0.5f)
            {
                heading = 2 * Math.Atan2(q.X, q.W);
                bank = 0;
            }
            else if (q.X * q.Y + q.Z * q.W == -0.5f)
            {
                heading = - 2 * Math.Atan2(q.X, q.W);
                bank = 0;
            }

            euler[0] = (float) heading;
            euler[1] = (float) attitude;
            euler[2] = (float)bank;

            return euler;
        }

        public static float[] MatrixToEuler(Matrix4 matrix)
        { 

            float [] euler = new float[3];
            
            double heading = Math.Atan2(matrix.M31, matrix.M11);
            double attitude = Math.Asin(matrix.M21);
            double bank = Math.Atan2(-matrix.M23, matrix.M22);

            if (matrix.M21 == 1)
            {
                heading = Math.Atan2(matrix.M13, matrix.M33);
                bank = 0;
            }
            else if (matrix.M21 == -1)
            {
                heading = Math.Atan2(matrix.M13, matrix.M33);
                bank = 0;
            }

            euler[0] = (float)heading;
            euler[1] = (float)attitude;
            euler[2] = (float)bank;

            return euler;
        }
    }
}
