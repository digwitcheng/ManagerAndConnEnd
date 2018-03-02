using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace Astar
{
    [Serializable]
    class MyPoint
    {
      private  int x;
      private  int y;

        public int Y
        {
            get { return y; }
            set { y = value; }
        }

        public int X
        {
            get { return x; }
            set { x = value; }
        }

        public MyPoint(MyPoint point)
        {
            this.x = point.x;
            this.y = point.y;

        }
        public MyPoint(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
       
        //public MyPoint(MyPoint point,int addSpeed)
        //{
        //    this.col = point.col;
        //    this.row = point.row;
        //    this.Speed += Speed;

        //}
        //public myPoint(float col, float row,Direction dir,int stopTime)
        //{
        //    this.col = col;
        //    this.row = row;
        //    this.direction = dir;
        //    this.stopTime = stopTime;
        //}
        //public myPoint(float col, float row)
        //{
        //    this.col = col;
        //    this.row = row;
        //}

         
    }
}
