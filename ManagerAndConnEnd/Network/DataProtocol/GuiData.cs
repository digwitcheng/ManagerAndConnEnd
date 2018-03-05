using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    class GuiData
    {
        //小车编号
        public ushort Num { get; set; }
        //小车的状态

        //{ free 0, needCharge 1, breakdown 2, cannotToDestination 3,carried 4 };
        public byte State
        {
            get;
            set;
        }
        public byte BeginX { get; set; }
        public byte BeginY { get; set; }
        public int DestX { get; set; }
        public int DestY { get; set; }
    }
}
