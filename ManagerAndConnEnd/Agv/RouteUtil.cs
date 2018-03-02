using AGV_V1._0.Util;
using Astar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    class RouteUtil
    {

        public static Random rand = new Random(3);//5,/4/4 //((int)DateTime.Now.Ticks);//随机数，随机产生坐标

        public static MyPoint RandPoint(ElecMap Elc)
        {
            int x = rand.Next(0, Elc.HeightNum - 1);
            int y = rand.Next(0, Elc.WidthNum - 1);
            while (!Elc.IsNodeAvailable(x, y))
            {
                x = rand.Next(0, Elc.HeightNum - 1);
                y = rand.Next(0, Elc.WidthNum - 1);
            }

            return new MyPoint(x, y);
        }
        public static MyPoint RandRealPoint(ElecMap Elc)
        {
            int minX=ConstDefine.minX;
            int maxX=ConstDefine.maxX;
            int minY = ConstDefine.minY;
            int maxY = ConstDefine.maxY;
            int x = rand.Next(minX, maxX+1);
            int y = rand.Next(minY,maxY+1);
            while (!Elc.IsNodeAvailable(x, y))
            {
                x = rand.Next(minX, maxX);
                y = rand.Next(minY, maxY);
            }
            return new MyPoint(x, y);
        }

        /// <summary>
        /// 把小车所在的节点设为占用状态
        /// </summary>
        public static void VehicleOcuppyNode(ElecMap Elc, Vehicle[] vehicle)
        {
            for (int p = 0; p < Elc.HeightNum; p++)
            {
                for (int q = 0; q < Elc.WidthNum; q++)
                {
                    Elc.mapnode[p, q].NodeCanUsed = -1;
                }
            }
            int count = vehicle.Length;
            for (int i = 0; i < count; i++)
            {
                Elc.mapnode[(vehicle[i].BeginX), (vehicle[i].BeginY)].NodeCanUsed = vehicle[i].Id;
            }
        }
        public int changeX(int X)
        {
            return (X ) / ConstDefine.g_NodeLength;
        }
        public int changeY(int Y)
        {
            return Y / ConstDefine.g_NodeLength;
        }


        /// <summary>
        ///// //检测冲突的节点，重新规划路线
        ///// </summary>
        //public void CheckeConflictNode(Vehicle[] vehicle, ElecMap Elc)
        //{
        //    //根据cost把小车从小到大排序
        //    //SortRoute();
        //    // VehicleOcuppyNode();
        //    int count = vehicle.Length;
        //    for (int i = 0; i < count - 1; i++)
        //    {
        //        if (vehicle[i].route == null)
        //        {
        //            continue;
        //        }
        //        for (int j = 0; j != i; j++)
        //        {

        //            if (vehicle[j].route == null)
        //            {
        //                continue;
        //            }
        //            if (vehicle[i].routeIndex < vehicle[i].route.Count - 3 && vehicle[j].routeIndex < vehicle[j].route.Count - 3)
        //           {
        //                int iCurrentIndex = vehicle[i].routeIndex + 2;
        //                int jCurrentIndex = vehicle[j].routeIndex + 2;
        //                float ix = vehicle[i].route[iCurrentIndex].col;
        //                float jx = vehicle[j].route[jCurrentIndex].col;
        //                float iy = vehicle[i].route[iCurrentIndex].row;
        //                float jy = vehicle[j].route[jCurrentIndex].row;
        //                //    System.Console.Write(Math.Abs(ix - jx) + Math.Abs(iy - jy)+" ");
        //                if (Math.Abs(ix-jx)+Math.Abs(iy-jy)<=1)
        //                {
        //                    Elc.mapnode[(int)vehicle[j].route[jCurrentIndex].col, (int)vehicle[j].route[jCurrentIndex].row].NodeCanUsed = false;
        //                    vehicle[j].SearchRouteThread(Elc);
        //                }
        //            }

        //        }

        //    }
        //}
        ///// <summary>
        ///// //根据cost把小车从大到小排序
        ///// </summary>
        //public void SortRoute()
        //{
        //    int count = vehicle.Length;
        //    for (int i = 0; i < count; i++)
        //    {
        //        Vehicle temp = vehicle[i];
        //        for (int j = i; j < count; j++)
        //        {
        //            if (vehicle[i].cost > vehicle[j].cost)
        //            {
        //                temp = vehicle[j];
        //                vehicle[j] = vehicle[i];
        //                vehicle[i] = temp;
        //            }
        //        }
        //        // vehicleSorted[i] = vehicle[index];
        //    }
        //}
    }
}
