using AGV_V1._0.Agv;
using Astar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    class MyLocation
    {

        public MyLocation(MyPoint point, int speed, Direction dir)
        {
            this.Point = point;
            this.speed = speed;
            this.Dir = dir;
        }
        public MyPoint Point { get; set; }
        public Direction Dir { get; set; }
        private int speed;
        public int Speed
        {

            get
            {
                return speed;
            }
            set
            {
                this.speed = value;
            }

        }
    }
}
