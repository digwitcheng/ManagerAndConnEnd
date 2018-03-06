using Agv.PathPlanning;
using AGV_V1._0.Agv;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    class AstarUtil
    {

        public const int Sequential = 0;    //顺序遍历
        public const int NoSolution = 2;    //无解决方案
        public const int Infinity = 0xfffffff;
        internal const int MaxLength = 6000;   //用于优先队列（Open表）的数组

        public static MyPoint[] dir = new MyPoint[4]{	
        new MyPoint( 0, 1),   //,Direction.RightDifficulty // East 0
	 //   new myPoint( 1, 1, ),   // South_East 1
	    new MyPoint( 1, 0),  //,Direction.DownDifficulty // South 2
	  //  new myPoint(1, -1 ),  // South_West 3
	    new MyPoint( 0, -1 ), //,Direction.LeftDifficulty // West 4
	 //   new myPoint( -1, -1 ), // North_West 5
        new MyPoint( -1, 0), //,Direction.UpDifficulty  // North 6
	 //   new myPoint( -1, 1)   // North_East 7
        };
        public static void push(Open q, Close[,] cls, int x, int y, float g)
        {    //向优先队列（Open表）中添加元素
            Close t;
            int i, mintag;
            cls[x, y].G = g;    //所添加节点的坐标
            cls[x, y].F = cls[x, y].G + cls[x, y].H;

            q.Array[q.length++] = (cls[x, y]);
            mintag = q.length - 1;
            for (i = 0; i < q.length - 1; i++)
            {
                if (q.Array[i].F < q.Array[mintag].F)
                {
                    mintag = i;
                }
            }
            t = q.Array[q.length - 1];
            q.Array[q.length - 1] = q.Array[mintag];
            q.Array[mintag] = t;    //将评价函数值最小节点置于队头
        }
        public static Direction getDirection(Close from, Close curPoint)
        {

            if (from.node.x - curPoint.node.x == 1)
            {
                return Direction.Up;// 3 North;
            }
            if (from.node.x - curPoint.node.x == -1)
            {
                return Direction.Down;// 1;//South;
            }
            if (from.node.y - curPoint.node.y == 1)
            {
                return Direction.Left;// 2;//West;
            }
            if (from.node.y - curPoint.node.y == -1)
            {
                return Direction.Right;// 0;// East;
            }
            return from.node.direction;
        }
        public static Close shift(Open q)
        {
            return q.Array[--q.length];
        }
       
    }
}
