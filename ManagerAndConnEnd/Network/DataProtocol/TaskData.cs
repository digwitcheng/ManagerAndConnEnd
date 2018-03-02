using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    class TaskData
    {
        //小车编号
        private int num;

        public int Num
        {
            get { return num; }
            set { num = value; }
        }
        //车的横坐标
        private int beginX;

        public int BeginX
        {
            get { return beginX; }
            set { beginX = value; }
        }
        //车的纵坐标
        private int beginY;

        public int BeginY
        {
            get { return beginY; }
            set { beginY = value; }
        }

        public int EndX { get; set; }
        public int EndY { get; set; }

        public bool Arrive { get; set; }
    }
}
